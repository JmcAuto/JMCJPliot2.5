/*****************************************************************************
 * Copyright 2017 The JmcAuto Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/control/controller/pid_stantly_controller.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <string>
#include <utility>
#include <vector>

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/log.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/math/quaternion.h"
#include "modules/common/time/time.h"
#include "modules/common/util/string_util.h"
#include "modules/control/common/control_gflags.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/common/math/vec2d.h"

namespace jmc_auto {
namespace control {

using jmc_auto::common::ErrorCode;
using jmc_auto::common::Point3D;
using jmc_auto::common::Status;
using jmc_auto::common::TrajectoryPoint;
using jmc_auto::common::VehicleStateProvider;
using jmc_auto::common::util::StrCat;

using jmc_auto::common::time::Clock;
using jmc_auto::common::PathPoint;
 // namespace
void PidStantlyController::ProcessLogs(const SimpleLateralDebug *debug,
                                const canbus::Chassis *chassis) {
  // StrCat supports 9 arguments at most.
  ADEBUG << "Process log" ;
   fprintf(steer_log_file_,
          "%.6f , %.6f , %.6f , %.6f , %.6f , %.6f , %.6f , "
         " %.6f , %.6f , %.6f ,\r\n",
          debug->lateral_error() , debug->heading_error(),
          debug->x_error() , debug->y_error(),
          debug->lateral_compensation(), debug->heading_compensation(),
          debug->steering_angle_cmd(), debug->steering_position(),
          debug->vp_point_distance(),
          debug->steering_target_position_error());

  return ;
  //ADEBUG << "Steer_Control_Detail: " << log_str;
}

PidStantlyController::PidStantlyController() : name_("PidStantly Controller") {
if (FLAGS_enable_csv_debug) {
  ADEBUG << "printf pidstantly csv log";
  time_t raw_time;
  char name_buffer[80];
  std::time(&raw_time);
  strftime(name_buffer, 80, "data/csv/steer_log_pid_stantly_%F_%H%M%S.csv",
           localtime(&raw_time));
  steer_log_file_ = fopen(name_buffer,"w");
  if (steer_log_file_ == nullptr)
  {
    AERROR << "Fail to open file:" << name_buffer ;
    FLAGS_enable_csv_debug = false ;
  }else
  {
    ADEBUG << "steer_log_file not empty" ;
   // steer_log_file_ << std::fixed;
   // steer_log_file_ << std::setprecision(6);
    fprintf(steer_log_file_,
              "lateral_error,"
              "heading_error,"
              "x_error,"
              "y_error,"
              "lateral_compensation,"
              "heading_compensation,"
              "steering_angle_cmd,"
              "steering_position,"
              "vp_point_distance,"
              "steering_target_position_error,"
             "\r\n");
    fflush(steer_log_file_);
  }
  }
  AINFO << "Using " << name_;
}

PidStantlyController::~PidStantlyController() { CloseLogFile(); }

bool PidStantlyController::LoadControlConf(const ControlConf *control_conf) {
  const auto &vehicle_param_ =
      common::VehicleConfigHelper::instance()->GetConfig().vehicle_param();
  //TODO:calibration中增加PidStantlyController配置参数
  ts_ = control_conf->pid_stantly_conf().ts();
  CHECK_GT(ts_, 0.0) << "[PidStantlyController] Invalid control update interval.";
 //车辆前端距车辆质心距离
  wheelbase_ = vehicle_param_.wheel_base();
  steer_transmission_ratio_ = vehicle_param_.steer_ratio();
  steering_max_degree_ = vehicle_param_.max_steer_angle() / M_PI * 180;
//预瞄距离
  lookahead =  control_conf->pid_stantly_conf().lookahead();
  lookback = control_conf->pid_stantly_conf().lookback();
  kp = control_conf->pid_stantly_conf().kp();
  ksoft = control_conf->pid_stantly_conf().ksoft();
  AINFO << "Load pid stantly conf file succeed!" ;
  return true;
}


Status PidStantlyController::Init(const ControlConf *control_conf) {
  control_conf_ = control_conf;
  if (!LoadControlConf(control_conf)) {
    AERROR << "failed to load control conf";
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR,
                  "failed to load control_conf");
  }
  //PID初始化





lateral_pid_controller_.Init(control_conf_->pid_stantly_conf().lateral_pid_conf());
reverse_pid_controller_.Init(control_conf_->pid_stantly_conf().reverse_pid_conf());
ADEBUG<< "Lat init succeed!";
  return Status::OK();
}

void PidStantlyController::CloseLogFile() {
  if (FLAGS_enable_csv_debug ) {
    if (steer_log_file_ != nullptr)
    {
      fclose(steer_log_file_);
      steer_log_file_ = nullptr;
    }
  }
}

void PidStantlyController::Stop() { CloseLogFile(); }

std::string PidStantlyController::Name() const { return name_; }

Status PidStantlyController::ComputeControlCommand(
    const localization::LocalizationEstimate *localization,
    const canbus::Chassis *chassis,
    const planning::ADCTrajectory *planning_published_trajectory,
    ControlCommand *cmd) {

    auto vehicle_state = VehicleStateProvider::instance() ;
    chassis_ = chassis;
    trajectory_analyzer_ = std::move(TrajectoryAnalyzer(planning_published_trajectory));
    SimpleLateralDebug *debug = cmd->mutable_debug()->mutable_simple_lat_debug();
    debug->Clear();

    debug->set_is_parking_mode(planning_published_trajectory->decision().main_decision().has_parking());
    debug->set_is_summon_mode(planning_published_trajectory->decision().main_decision().has_summon());
    //获取车辆行驶航向
    UpdateDrivingOrientation();
    //计算横向航向误差
    UpdateErrors(debug);
    //计算横向误差->方向盘角度
   // double kp_v = 0 ;//--------------根据速度v实时调整比例系数
   // kp_v = vehicle_state->linear_velocity() * kp ;
    //double v_ = ksoft + vehicle_state->linear_velocity();
    //v_ = std::max(v_, 0.01); //------防止v_为0
    //lateral_compensation = -std::atan2(kp_v*lateral_error/v_)*steer_transmission_ratio_*rad_to_deg;
    if (vehicle_state->gear() == canbus::Chassis::GEAR_REVERSE)
    {
    lateral_compensation = reverse_pid_controller_.Control(lateral_error,ts_);
    }
    lateral_compensation = lateral_pid_controller_.Control(lateral_error,ts_);
    //lateral_compensation = lateral_compensation * steer_transmission_ratio_ * rad_to_deg ;
    heading_compensation = heading_error * steer_transmission_ratio_*rad_to_deg;
    ADEBUG << "lateral_compensation " << lateral_compensation ;
    ADEBUG << "heading_compensation " << heading_compensation ;
    steering_angle = lateral_compensation ;
    ComputeSteeringAngle(steering_angle) ;
    cmd->set_steering_angle(steering_angle) ;

    ADEBUG << "gear " << vehicle_state->gear() ;
    ADEBUG << "CAR speed" << vehicle_state->linear_velocity() ;
    ADEBUG << "steering angle command " << steering_angle ;
    cmd->set_steering_rate(FLAGS_steer_angle_rate);
    debug->set_steering_position(chassis->steering_percentage()/steer_transmission_ratio_/180*M_PI);
    debug->set_ref_speed(vehicle_state->linear_velocity());
    debug->set_steering_target_position_error(steering_angle - chassis->steering_percentage()) ;//增加车轮目标位置和实际位置误差输出信息
    debug->set_steering_angle_cmd(steering_angle);
    debug->set_lateral_compensation(lateral_compensation);
    debug->set_heading_compensation(heading_compensation);
    ADEBUG << "steering_position:" << debug->steering_position();
    ADEBUG << "steering_angle :" << steering_angle ;
    ProcessLogs(debug, chassis);
    return Status::OK();

}

Status PidStantlyController::Reset() {
  lateral_pid_controller_.Reset();
  return Status::OK();
}

/***********************************************************
 * 更新横向误差和航向误差
 * front_com---------车辆前轴中心位置
************************************************************/
void PidStantlyController::UpdateErrors(SimpleLateralDebug *debug)
{
  auto vehicle_state = VehicleStateProvider::instance();
  if (debug->is_parking_mode() && vehicle_state->gear() == canbus::Chassis::GEAR_REVERSE)
  {
    ADEBUG << "parking mode ,R" ;
    ComputeErrors(vehicle_state->x(),vehicle_state->y(),
                  driving_orientation,lookback,trajectory_analyzer_,debug);
  }else
  {
    ADEBUG << "UpdateErrors D" ;
    const auto &front_com = vehicle_state->ComputeCOMPosition(wheelbase_);
    ComputeErrors(front_com.x(),front_com.y(),
                         driving_orientation,lookback,
                         trajectory_analyzer_, debug);
  }
    ADEBUG << "UpdateErrors succeed!";
}

/***********************************************************
  计算横向，航向误差
  x--------车辆世界坐标，泊车时为后轴中心x坐标，前进时为前轴中心x坐标
  y--------车辆世界坐标，泊车时为后轴中心y坐标，前进时为前轴中心y坐标
  theta----车辆行驶方向航向角
  preview_distance-------预瞄距离
  preview_position-------车辆预瞄位置
  preview_point----------预瞄点
  lateral_error----------横向误差
  heading_error----------航向误差
  match_nearest_point----车辆当前位置匹配到的最近点
*/
void PidStantlyController::ComputeErrors(
    const double x, const double y, const double theta,
    const double preview_distance , const TrajectoryAnalyzer &trajectory_analyzer,
    SimpleLateralDebug *debug)
  {
    auto vehicle_state = VehicleStateProvider::instance();
    if ((!debug->is_parking_mode())&&(!debug->is_summon_mode()))
    {
      trajectory_analyzer_.TrajectoryTransformToCOM(wheelbase_);
    }
    auto match_nearest_point = trajectory_analyzer.QueryMatchedPathPoint(x,y);
    auto match_theta = match_nearest_point.theta();
    //1.计算预瞄位置
    common::math::Vec2d preview_position ;
    preview_position.set_x(x + preview_distance*std::cos(theta));
    preview_position.set_y(y + preview_distance*std::sin(theta));
    //2 找轨迹点中距离预瞄点最近的点
    const auto preview_point = trajectory_analyzer_.QueryMatchedPathPoint(preview_position.x(),preview_position.y());
    double preview_point_theta = preview_point.theta();
    if (vehicle_state->gear()==canbus::Chassis::GEAR_REVERSE)
    {
      preview_point_theta = common::math::NormalizeAngle(preview_point_theta + M_PI );
      match_theta = common::math::NormalizeAngle(match_theta + M_PI );
    }

    //3.计算横向误差
    lateral_error = -(preview_point.x() - preview_position.x())*std::sin(preview_point_theta)
                              + (preview_point.y() - preview_position.y())*std::cos(preview_point_theta);
    //4.计算航向误差
    heading_error = common::math::NormalizeAngle(preview_point_theta-theta) ;
    //倒档下航向误差和横向误差相反
    if (vehicle_state->gear() == canbus::Chassis::GEAR_REVERSE)
    {
      ADEBUG << "R,reverse heading_error and lateral_error" ;
      heading_error = -heading_error ;
      lateral_error = -lateral_error ;
    }
    ADEBUG << "heading_error " << heading_error ;
    ADEBUG << "lateral_error " << lateral_error ;


    double x_error = x - match_nearest_point.x() ;
    double y_error = y - match_nearest_point.y() ;
    ADEBUG << "x_error y_error heading_error " << x_error << " "
                                               << y_error << " "
                                               << theta - match_theta ;

    double vp_point_distance = std::sqrt(x_error*x_error + y_error*y_error) ;
    ADEBUG << "vp_point_distance " << vp_point_distance ;
    debug->set_lateral_error(lateral_error);
    debug->set_heading_error(heading_error);
    debug->set_x_error(x_error);
    debug->set_y_error(y_error);
    debug->set_theta_error(theta - match_theta);
    debug->set_vp_point_distance(vp_point_distance);
    debug->set_curvature(preview_point.kappa());
    debug->set_ref_heading(preview_point_theta);
    debug->set_heading(driving_orientation);
    debug->set_steering_target_position_error(theta - match_theta);
    return ;
 }

void PidStantlyController::UpdateDrivingOrientation() {
  auto vehicle_state = VehicleStateProvider::instance();
  driving_orientation = vehicle_state->heading();
  // Reverse the driving direction if the vehicle is in reverse mode
  if (FLAGS_reverse_heading_control) {
    if (vehicle_state->gear() == canbus::Chassis::GEAR_REVERSE) {
      driving_orientation =
          common::math::NormalizeAngle(driving_orientation + M_PI);
      // Update Matrix_b for reverse mode
      ADEBUG << "Matrix_b changed due to gear direction";
    }
  }
}

void PidStantlyController::ComputeSteeringAngle(double & steering_angle_)
{
  // steering_angle = feedback + feedforward ;
   //限制在方向盘最大转角内
 //  steering_angle = common::math::Clamp(steering_angle,-steering_max_degree_,steering_max_degree_);
   ADEBUG << "origin steering_angle" << steering_angle_ ;
   //数字滤波
//   steering_angle_ = digital_filter_.Filter(steering_angle);
//   ADEBUG << "after digital filter steering_angle" << steering_angle_ ;
   //和底盘反馈当前方向盘角度对比,前后差值不能超过90度
   double steering_angle_diff = steering_angle_ - chassis_->steering_percentage() ;
   steering_angle_diff = common::math::Clamp(steering_angle_diff,-60.0,60.0) ;
   steering_angle_ = chassis_->steering_percentage() + steering_angle_diff ;
   ADEBUG << "after chassis limited steering angle " << steering_angle_ ;
   //均值滤波
   // steering_angle_ = lateral_error_filter_.Update(steering_angle_);
   //限制角度防止底盘执行超调
   steering_angle = common::math::Clamp(steering_angle_,-482.0,482.0) ;
   return ;
}

}  // namespace control
}  // namespace jmc_auto
