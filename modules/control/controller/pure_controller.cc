/*opyright 2017 The JmcAuto Authors. All Rights Reserved.
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

#include "modules/control/controller/pure_controller.h"

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
void PureController::ProcessLogs(const SimpleLateralDebug *debug,
                                const canbus::Chassis *chassis) {
  // StrCat supports 9 arguments at most.
  ADEBUG << "Process log" ;
   fprintf(steer_log_file_,
          "%.6f , %.6f , %.6f , %.6f , %.6f , "
         " %.6f , %.6f , %.6f ,%.6f ,%.6f ,%.6f ,%.6f ,\r\n",
          debug->lateral_error() , debug->heading_error(),
          debug->x_error() , debug->y_error(),
          debug->steering_angle_cmd(), debug->steering_position(),
          debug->v_pre_point_distance(),
          debug->alpha(),
          debug->steering_target_position_error(),
          debug->ref_heading(),
          debug->heading(),
          debug->curvature());


  return ;
  //ADEBUG << "Steer_Control_Detail: " << log_str;
}

PureController::PureController() : name_("Pure Controller") {
if (FLAGS_enable_csv_debug) {
  ADEBUG << "printf pure csv log";
  time_t raw_time;
  char name_buffer[80];
  std::time(&raw_time);
  strftime(name_buffer, 80, "data/csv/steer_log_pure_%F_%H%M%S.csv",
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
              "lateral_error,"              // 纯跟踪算法未用该值，但可输出，作为跟踪评价指标(计算匹配点与车辆当前位置的横向误差)
              "heading_error,"          // 纯跟踪算法未用该值，但可输出，作为跟踪评价指标，同上
              "x_error,"                        //  应该为匹配点与车辆当前位置(后轴中心)在笛卡尔坐标系下的x坐标差值
              "y_error,"                        //  同上
              "steering_angle_cmd,"//方向盘转角作为控制量，需要将前轮转角进行弧度制转换，并且乘以角传动比
              "steering_position,"
              "v_pre_point_distance,"  //  车辆当前点（后轴中心坐标）与预瞄点的连线距离，用于纯跟踪算法计算前轮转角
              "alpha,"                            //  车辆当前点（后轴中心坐标）和预瞄点的连线的航向角与车辆航向角的差值，用于纯跟踪算法计算前轮转角
              "steering_target_position_error,"
              "ref_heading,"           //预瞄点的航向
              "heading,"               //车辆的行驶航向
              "curvature,"             //预瞄点的参考曲率
             "\r\n");
    fflush(steer_log_file_);
  }
  }
  AINFO << "Using " << name_;
}

PureController::~PureController() { CloseLogFile(); }
/**********************************************以上部分都是对日志（log）文件的处理*********************************************/

/********************************************函数3：判断加载Pure控制器的配置参数是否成功*********************************************/
bool PureController::LoadControlConf(const ControlConf *control_conf) {
  const auto &vehicle_param_ =
      common::VehicleConfigHelper::instance()->GetConfig().vehicle_param();
  //TODO:calibration中增加PureController配置参数
  ts_ = control_conf->pure_conf().ts();
  CHECK_GT(ts_, 0.0) << "[PureController] Invalid control update interval.";
 //车辆前端距车辆质心距离
  wheelbase_ = vehicle_param_.wheel_base();
  steer_transmission_ratio_ = vehicle_param_.steer_ratio();
  steering_max_degree_ = vehicle_param_.max_steer_angle() / M_PI * 180;
//在本纯跟踪算法中，预瞄的思想是根据车辆当前点找轨迹上的匹配点，然后在匹配点往后索引点，确定预瞄点
//在pure控制器中，分了三个场景下的预瞄，泊车、召唤、行车，配置参数在控制模块中已加载
//前进时的预瞄距离和倒车时的预瞄距离，仿真模型中的Id(暂时不用这种方法)
//lookahead =  control_conf->pure_conf().lookahead();
//lookback =  control_conf->pure_conf().lookback();
  AINFO << "Load pure conf file succeed!" ;
  return true;
}

/*******************************************1：初始化控制器：加载Pure控制器的配置参数********************************************/
Status PureController::Init(const ControlConf *control_conf) {
  control_conf_ = control_conf;
  // 如果无法加载配置文件，提示加载失败，通过函数3来实现对配置文件的加载
  if (!LoadControlConf(control_conf)) {
    AERROR << "failed to load control conf";
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR,
                  "failed to load control_conf");
  }
  ADEBUG << "Lat init succeed!";
  return Status::OK();
}

// 关闭日志文件
void PureController::CloseLogFile() {
  if (FLAGS_enable_csv_debug ) {
    if (steer_log_file_ != nullptr)
    {
      fclose(steer_log_file_);
      steer_log_file_ = nullptr;
    }
  }
}

void PureController::Stop() { CloseLogFile(); }

std::string PureController::Name() const { return name_; }

/********************************************2：计算控制器Pure的控制指令*******************************************/
Status PureController::ComputeControlCommand(
    const localization::LocalizationEstimate *localization,
    const canbus::Chassis *chassis,
    const planning::ADCTrajectory *planning_published_trajectory,
    ControlCommand *cmd) {

    auto vehicle_state = VehicleStateProvider::instance() ;
    chassis_ = chassis;
    trajectory_analyzer_ = std::move(TrajectoryAnalyzer(planning_published_trajectory));
    SimpleLateralDebug *debug = cmd->mutable_debug()->mutable_simple_lat_debug();
    trajectory_message_ = planning_published_trajectory;
    debug->Clear();

    debug->set_is_parking_mode(planning_published_trajectory->decision().main_decision().has_parking());
    debug->set_is_summon_mode(planning_published_trajectory->decision().main_decision().has_summon());
    if (planning_published_trajectory->decision().main_decision().has_parking())
    {
      preview_window_ = control_conf_->pure_conf().parking_preview_window() ;
    }else if(debug->is_summon_mode())
    {
      preview_window_ = control_conf_->pure_conf().summon_preview_window() ;
    }else
    {
      preview_window_ = control_conf_->pure_conf().driving_preview_window() ;
    }
    //获取车辆行驶航向
    UpdateDrivingOrientation();
    //计算横向、航向误差
    UpdateErrors(debug);
    //计算横向误差->方向盘角度
    // 纯跟踪算法：计算方向盘转角，作为最终的控制量(解决泊车泊反问题)
    if (vehicle_state->gear() == canbus::Chassis::GEAR_DRIVE)
    {
      steering_angle = std::atan(2*wheelbase_ *(std::sin(debug->alpha()))/debug->v_pre_point_distance()) * steer_transmission_ratio_ * rad_to_deg ;
    }else if(vehicle_state->gear() == canbus::Chassis::GEAR_REVERSE)
    {
      steering_angle = -std::atan(2*wheelbase_ *(std::sin(debug->alpha()))/debug->v_pre_point_distance()) * steer_transmission_ratio_ * rad_to_deg ;
    }
    //方法1：处理行车过程中，车辆到达终点时，方向盘抖动问题-----------------------------------------修改1：方向盘抖动问题
    if (vehicle_state->gear() == canbus::Chassis::GEAR_DRIVE){
      if ((trajectory_message_->trajectory_point_size() - 120) <= 0)     //  判断开始抖动的点
      {
        steering_angle = 0.05;                                           //  固定的steering_angle 待确定
      }
    }
    //方法2：处理行车过程中，车辆到达终点时，方向盘抖动问题(吴老师)
    // if (vehicle_state->gear() == canbus::Chassis::GEAR_DRIVE) {
    //    int fixed_index = 0;
    //    if (fixed_index = trajectory_message_->trajectory_point_size() - 500)     // fixed_index 待确定
    //    {
    //        steering_angle = std::atan(2*wheelbase_ *(std::sin(debug->alpha()))/0.12) * steer_transmission_ratio_ * rad_to_deg ;                                                                                                      //  固定的steering_angle 待确定
    //    }
    //  }
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

    ADEBUG << "steering_position:" << debug->steering_position();
    ADEBUG << "steering_angle :" << steering_angle ;
    ProcessLogs(debug, chassis);
    return Status::OK();

}

Status PureController::Reset() {
  return Status::OK();
}

/****************************************************以下函数均为被调用的函数*******************************************************/
/********************************************函数4：更新横向误差和航向误差*********************************************/
/***********************************************************
 * 更新横向误差和航向误差
 * vehicle_state->x()，vehicle_state->y()----------车辆后轴中心坐标，车辆位置默认是后轴中心位置
 * 注意在pure控制器中，前进和倒退均是以车辆后轴中心位置作为车辆当前位置
************************************************************/
void PureController::UpdateErrors(SimpleLateralDebug *debug)
{
  auto vehicle_state = VehicleStateProvider::instance();
  if (debug->is_parking_mode() && vehicle_state->gear() == canbus::Chassis::GEAR_REVERSE)
  {
    ADEBUG << "parking mode ,R" ;
    ComputeErrors(vehicle_state->x(),vehicle_state->y(),
                  driving_orientation,trajectory_analyzer_,debug);
  }else
  {
    ADEBUG << "UpdateErrors D" ;
    ComputeErrors(vehicle_state->x(),vehicle_state->y(),
                         driving_orientation, trajectory_analyzer_, debug);
  }
    ADEBUG << "UpdateErrors succeed!";
}

/***************************************函数5：计算横向误差和航向误差(被函数4调用)*****************************************/
// 注意：在纯跟踪算法中，并没有用到ed、ephi，计算当前点与匹配点的ed、ephi的目的是作为跟踪评价指标
/***********************************************************
  计算横向，航向误差
  x--------车辆世界坐标，前进和倒退均是以车辆后轴中心位置作为车辆当前位置
  y--------车辆世界坐标，前进和倒退均是以车辆后轴中心位置作为车辆当前位置
  theta----车辆行驶方向航向角
  preview_point----------预瞄点
  lateral_error----------横向误差
  heading_error----------航向误差
  match_nearest_point----车辆当前位置匹配到的最近点
  v_pre_point_distance -----车辆当前点（后轴中心坐标）与预瞄点的连线距离，用于纯跟踪算法计算前轮转角
*/
void PureController::ComputeErrors(
    const double x, const double y, const double theta,
    const TrajectoryAnalyzer &trajectory_analyzer,
    SimpleLateralDebug *debug)
  {
    auto vehicle_state = VehicleStateProvider::instance();
    if ((!debug->is_parking_mode())&&(!debug->is_summon_mode()))
    {
      trajectory_analyzer_.TrajectoryTransformToCOM(0);
    }
    // 泊车时选取预瞄点
    /*
    1、在参考轨迹上寻找与车辆当前位置的最近的匹配点
    2、找出匹配点后，再在匹配点的前方取点作为预瞄点，preview_window=120，就认为是取匹配点前方的第120个点作为预瞄点(通过索引找点)
    3、选取预瞄点即preview_point
    */
   // 1、根据车辆当前点找匹配点
    auto match_nearest_point = trajectory_analyzer.QueryMatchedPathPoint(x,y);
    auto match_theta = match_nearest_point.theta();
   // 2、根据匹配点，通过索引找预瞄点，输入的是后轴中心和预瞄窗口；
   // 该函数的实现就是先确定匹配点及索引，然后获取预瞄点的索引，即最终确定
    auto preview_point = trajectory_analyzer.QueryPreviewPointByPosition(x,y,preview_window_);
   // 增加预瞄点曲率自适应处理，设置一个曲率阈值，通过if语句来实现(吴老师)-----------------------修改2：预瞄点曲率自适应(泊车和行车待区分)
/*    if (preview_point.path_point().kappa()> 0.10)
    {
        int preview_window_auto_ = 50;
        preview_point = trajectory_analyzer.QueryPreviewPointByPosition(x, y, preview_window_auto_);
    }
*/
    double preview_point_theta = preview_point.path_point().theta() ;            // 预瞄点的航向角
    if (vehicle_state->gear()==canbus::Chassis::GEAR_REVERSE)                    // 如果是倒车，目标点的航向需要+pi
    {
      preview_point_theta = common::math::NormalizeAngle(preview_point_theta + M_PI );
      match_theta = common::math::NormalizeAngle(match_theta + M_PI );
    }

    //3.计算横向误差(计算匹配点与车辆当前位置的横向误差，作为跟踪的评价指标)
    lateral_error = -(x-match_nearest_point.x())*std::sin(match_theta)
                              + (y-match_nearest_point.y())*std::cos(match_theta);
    //4.计算航向误差(计算车辆当前位置与匹配点的航向误差，作为跟踪的评价指标)
    heading_error = common::math::NormalizeAngle(theta-match_theta) ;
    // 5、计算车辆当前点（后轴中心坐标）与预瞄点的连线的航向角
    double pre_v_theta =  std::atan2((preview_point.path_point().y() - y),(preview_point.path_point().x() - x));
    double alpha = pre_v_theta - theta;
    //倒档下航向误差、横向误差、alpha相反
    if (vehicle_state->gear() == canbus::Chassis::GEAR_REVERSE)
    {
      ADEBUG << "R,reverse heading_error and lateral_error" ;
      heading_error = -heading_error ;
      lateral_error = -lateral_error ;
      alpha = -alpha;
    }
    ADEBUG << "heading_error " << heading_error ;
    ADEBUG << "lateral_error " << lateral_error ;

    // 6、v_pre_point_distance 车辆当前点（后轴中心坐标）与预瞄点的连线距离，用于纯跟踪算法计算前轮转角
    double x_error = x - preview_point.path_point().x() ;
    double y_error = y - preview_point.path_point().y() ;
    ADEBUG << "x_error y_error heading_error " << x_error << " "
                                               << y_error << " "
                                               << theta - match_theta ;

    double v_pre_point_distance = std::sqrt(x_error*x_error + y_error*y_error) ;
    ADEBUG << "v_pre_point_distance " << v_pre_point_distance ;
    debug->set_lateral_error(lateral_error);
    debug->set_heading_error(heading_error);
    debug->set_x_error(x_error);
    debug->set_y_error(y_error);
    debug->set_theta_error(theta - match_theta);
    debug->set_alpha(pre_v_theta - theta);
    debug->set_v_pre_point_distance(v_pre_point_distance);
    debug->set_curvature(preview_point.path_point().kappa());
    debug->set_ref_heading(preview_point_theta);
    debug->set_heading(driving_orientation);
    debug->set_steering_target_position_error(theta - match_theta);
    return ;
 }

/********************************************函数6：更新车辆的行驶航向*********************************************/
void PureController::UpdateDrivingOrientation() {
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

/********************************************函数7：设置方向盘转角转向约束*********************************************/
void PureController::ComputeSteeringAngle(double & steering_angle_)
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
