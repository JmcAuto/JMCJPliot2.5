/******************************************************************************
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

#include "modules/control/controller/mpc_controller.h"

#include <algorithm>
#include <iomanip>
#include <utility>
#include <vector>
#include <cstdio>

#include "Eigen/LU"

#include "modules/common/log.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/math/mpc_solver.h"
#include "modules/common/time/time.h"
#include "modules/common/util/string_util.h"
#include "modules/control/common/control_gflags.h"
#include "modules/localization/common/localization_gflags.h"

namespace jmc_auto {
namespace control {

using jmc_auto::common::ErrorCode;
using jmc_auto::common::Status;
using jmc_auto::common::TrajectoryPoint;
using jmc_auto::common::VehicleStateProvider;
using jmc_auto::common::time::Clock;
using Matrix = Eigen::MatrixXd;
//using jmc_auto::common::VehicleConfigHelper;

/* ****TODO(QiL): Add debug information，被ComputeControlCommand()调用*****/
void MPCController::ProcessLogs(const SimpleMPCDebug *debug,
                                const canbus::Chassis *chassis) {
   fprintf(mpc_log_file_,
         "%.6f , %.6f , %.6f , %.6f , %.6f , %.6f ,%.6f ,%.6f ,%.6f ,%.6f ,%.6f ,%.6f ,"
         " %.6f , %.6f , %.6f ,%.6f ,%.6f ,%.6f ,%.6f ,%.6f ,%.6f ,%.6f ,%.6f ,\r\n",
          debug->lateral_error(), debug->heading_error(),
          debug->x_error(),debug->y_error(),
          debug->steering_angle_cmd(), debug->steering_position(),debug->steering_target_position_error(),
          debug->steer_angle_feedforward(),debug->steer_angle_feedback(),
          debug->ref_heading(),debug->heading(),
          debug->curvature(),
          debug->speed_cmd(),debug->station_error(),
          debug->speed_error(),debug->station_reference(),
          debug->speed_reference(),debug->acceleration_reference(),
          debug->current_station(),debug->vehicle_speed(),
          debug->speed_feedback_closeloop(),debug->speed_real(),
          debug->vehicle_acceleration());
  return ;
  //ADEBUG << "Mpc_Control_Detail: " << log_str;
}

/**********创建CSV文件**********/
MPCController::MPCController() : name_("MPC Controller") {
  if (FLAGS_enable_csv_debug) {
  ADEBUG << "printf mpc csv log";
  time_t raw_time;
  char name_buffer[80];
  std::time(&raw_time);
  strftime(name_buffer, 80, "data/csv/mpc_log_%F_%H%M%S.csv",
           localtime(&raw_time));
  mpc_log_file_ = fopen(name_buffer,"w");
  if (mpc_log_file_ == nullptr)
  {
    AERROR << "Fail to open file:" << name_buffer ;
    FLAGS_enable_csv_debug = false ;
  }else{
    ADEBUG << "mpc_log_file not empty" ;
    fprintf(mpc_log_file_,
             "lateral_error,"
             "heading_error,"
             "x_error,"
             "y_error,"
             "steering_angle_cmd,"
             "steering_position,"
             "steering_target_position_error,"
             "steer_angle_feedforward,"
             "steer_angle_feedback,"
             "ref_heading,"
             "heading,"
             "curvature,"
             "speed_cmd,"
             "station_error,"
             "speed_error,"
             "station_reference,"
             "speed_reference,"
             "acceleration_reference,"
             "current_station,"
             "vehicle_speed,"
             "speed_feedback_closeloop,"
             "speed_real,"
             "vehicle_acceleration,"
             "\r\n");
    fflush(mpc_log_file_);
  }
  }
  AINFO << "Using " << name_;
}
void MPCController::CloseLogFile() {
  if (FLAGS_enable_csv_debug ) {
    if (mpc_log_file_ != nullptr)
    {
      fclose(mpc_log_file_);
      mpc_log_file_ = nullptr;
    }
  }
}
MPCController::~MPCController() { CloseLogFile(); }
void MPCController::Stop() { CloseLogFile(); }
std::string MPCController::Name() const { return name_; }

/**********加载MPC的配置参数**********/
bool MPCController::LoadControlConf(const ControlConf *control_conf) {
  if (!control_conf) {
    AERROR << "[MPCController] control_conf == nullptr";
    return false;
  }
  vehicle_param_ =
             common::VehicleConfigHelper::instance()->GetConfig().vehicle_param();
  ts_ = control_conf->mpc_controller_conf().ts();
  CHECK_GT(ts_, 0.0) << "[MPCController] Invalid control update interval.";
  cf_ = control_conf->mpc_controller_conf().cf();
  cr_ = control_conf->mpc_controller_conf().cr();
  wheelbase_ = vehicle_param_.wheel_base();
  steer_ratio_ = vehicle_param_.steer_ratio();
  steer_single_direction_max_degree_ =
      vehicle_param_.max_steer_angle() * 180 / M_PI;
  max_lat_acc_ = control_conf->mpc_controller_conf().max_lateral_acceleration();
  wheel_single_direction_max_degree_ =
      steer_single_direction_max_degree_ / steer_ratio_ / 180 * M_PI;
  max_acceleration_ = vehicle_param_.max_acceleration();
  max_deceleration_ = vehicle_param_.max_deceleration();

  const double mass_fl = control_conf->mpc_controller_conf().mass_fl();
  const double mass_fr = control_conf->mpc_controller_conf().mass_fr();
  const double mass_rl = control_conf->mpc_controller_conf().mass_rl();
  const double mass_rr = control_conf->mpc_controller_conf().mass_rr();
  const double mass_front = mass_fl + mass_fr;
  const double mass_rear = mass_rl + mass_rr;
  mass_ = mass_front + mass_rear;

  lf_ = wheelbase_ * (1.0 - mass_front / mass_);
  lr_ = wheelbase_ * (1.0 - mass_rear / mass_);
  iz_ = lf_ * lf_ * mass_front + lr_ * lr_ * mass_rear;

  mpc_eps_ = control_conf->mpc_controller_conf().eps();
  mpc_max_iteration_ = control_conf->mpc_controller_conf().max_iteration();
  throttle_deadzone_ = control_conf->mpc_controller_conf().throttle_deadzone();
  brake_deadzone_ = control_conf->mpc_controller_conf().brake_deadzone();

  minimum_speed_protection_ = control_conf->minimum_speed_protection();
  standstill_acceleration_ =
      control_conf->mpc_controller_conf().standstill_acceleration();

  LoadControlCalibrationTable(control_conf->mpc_controller_conf());
  AINFO << "MPC conf loaded";
  return true;
}

/**********初始化车辆参数**********/
void MPCController::LogInitParameters() {
  AINFO << name_ << " begin.";
  AINFO << "[MPCController parameters]"
        << " mass_: " << mass_ << ","
        << " iz_: " << iz_ << ","
        << " lf_: " << lf_ << ","
        << " lr_: " << lr_;
}

/**********初始化滤波器**********/
void MPCController::InitializeFilters(const ControlConf *control_conf) {
  // Low pass filter
  std::vector<double> den(3, 0.0);
  std::vector<double> num(3, 0.0);
  common::LpfCoefficients(
      ts_, control_conf->mpc_controller_conf().cutoff_freq(), &den, &num);
  digital_filter_.set_coefficients(den, num);
  lateral_error_filter_ = common::MeanFilter(static_cast<std::uint_fast8_t>(
      control_conf->mpc_controller_conf().mean_filter_window_size()));
  heading_error_filter_ = common::MeanFilter(static_cast<std::uint_fast8_t>(
      control_conf->mpc_controller_conf().mean_filter_window_size()));
}

/**********初始化MPC控制器**********/
Status MPCController::Init(const ControlConf *control_conf) {
  //int q_param_size = 0.0;
  if (!LoadControlConf(control_conf)) {
    AERROR << "failed to load control conf";
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR,
                  "failed to load control_conf");
  }
  speed_controller_input_limit = control_conf -> lon_controller_conf().speed_controller_input_limit();
  speed_pid_controller_.Init(control_conf -> lon_controller_conf().low_speed_pid_conf());
  // Matrix init operations.
  matrix_a_ = Matrix::Zero(basic_state_size_, basic_state_size_);
  matrix_ad_ = Matrix::Zero(basic_state_size_, basic_state_size_);
  matrix_a_(0, 1) = 1.0;
  matrix_a_(1, 2) = (cf_ + cr_) / mass_;
  matrix_a_(2, 3) = 1.0;
  matrix_a_(3, 2) = (lf_ * cf_ - lr_ * cr_) / iz_;
  matrix_a_(4, 5) = 1.0;
  matrix_a_(5, 5) = 0.0;
  // TODO(QiL): expand the model to accommodate more combined states.

  matrix_a_coeff_ = Matrix::Zero(basic_state_size_, basic_state_size_);
  matrix_a_coeff_(1, 1) = -(cf_ + cr_) / mass_;
  matrix_a_coeff_(1, 3) = (lr_ * cr_ - lf_ * cf_) / mass_;
  matrix_a_coeff_(2, 3) = 1.0;
  matrix_a_coeff_(3, 1) = (lr_ * cr_ - lf_ * cf_) / iz_;
  matrix_a_coeff_(3, 3) = -1.0 * (lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / iz_;

  matrix_b_ = Matrix::Zero(basic_state_size_, controls_);
  matrix_bd_ = Matrix::Zero(basic_state_size_, controls_);
  matrix_b_(1, 0) = cf_ / mass_;
  matrix_b_(3, 0) = lf_ * cf_ / iz_;
  matrix_b_(4, 1) = 0.0;
  matrix_b_(5, 1) = -1.0;
  matrix_bd_ = matrix_b_ * ts_;

  matrix_c_ = Matrix::Zero(basic_state_size_, 1);
  matrix_c_(5, 0) = 1.0;
  matrix_cd_ = Matrix::Zero(basic_state_size_, 1);

  matrix_state_ = Matrix::Zero(basic_state_size_, 1);
  matrix_k_ = Matrix::Zero(1, basic_state_size_);

  matrix_r_ = Matrix::Identity(controls_, controls_);

  matrix_q_ = Matrix::Zero(basic_state_size_, basic_state_size_);
  int r_param_size = control_conf->mpc_controller_conf().matrix_r_size();

  for (int i = 0; i < r_param_size; ++i) {
    matrix_r_(i, i) = control_conf->mpc_controller_conf().matrix_r(i);
  }

  int q_param_size = control_conf->mpc_controller_conf().matrix_q_size();
  if (basic_state_size_ != q_param_size) {
    const auto error_msg = common::util::StrCat(
        "MPC controller error: matrix_q size: ", q_param_size,
        " in parameter file not equal to basic_state_size_: ",
        basic_state_size_);
    AERROR << error_msg;
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, error_msg);
  }
  for (int i = 0; i < q_param_size; ++i) {
    matrix_q_(i, i) = control_conf->mpc_controller_conf().matrix_q(i);
  }

  // Update matrix_q_updated_ and matrix_r_updated_
  matrix_r_updated_ = matrix_r_;
  matrix_q_updated_ = matrix_q_;

  InitializeFilters(control_conf);
  //LoadMPCGainScheduler(control_conf->mpc_controller_conf());
  LogInitParameters();
  AINFO << "[MPCController] init done!";
  return Status::OK();
}

/**********将前轮转角转为方向盘转角**********/
double MPCController::Wheel2SteerToSteerAngle(const double wheel_angle) {
  return wheel_angle* steer_ratio_ * rad_to_deg ;
}

// 此处删除了MPC的增益序列，对航向误差、横向误差、前馈等的增益

/**********计算横纵向控制指令**********/
Status MPCController::ComputeControlCommand(
    const localization::LocalizationEstimate *localization,
    const canbus::Chassis *chassis,
    const planning::ADCTrajectory *planning_published_trajectory,
    ControlCommand *cmd) {
  trajectory_analyzer_ =
      std::move(TrajectoryAnalyzer(planning_published_trajectory));
  trajectory_message_ = planning_published_trajectory;
  auto vehicle_state = VehicleStateProvider::instance() ;
  chassis_ = chassis;
  SimpleMPCDebug *debug = cmd->mutable_debug()->mutable_simple_mpc_debug();
  debug->Clear();
  // 泊车、召唤模式传给debug
  debug->set_is_parking_mode(planning_published_trajectory->decision().main_decision().has_parking());
  debug->set_is_summon_mode(planning_published_trajectory->decision().main_decision().has_summon());
  ADEBUG << "parking mode" << debug->is_parking_mode();
  ADEBUG << "summon mode" << debug->is_summon_mode();
  double speed_cmd = 0.0;
  double steer_angle_feedback = 0.0;
  int q_param_size = control_conf_->mpc_controller_conf().matrix_q_size();
  // 设置行车和泊车控制逻辑
  if (planning_published_trajectory->decision().main_decision().has_parking()|| planning_published_trajectory->decision().main_decision().has_summon())
  {
    ADEBUG << "Parking mode" ;
     //将轨迹点转化到质心
    trajectory_analyzer_.TrajectoryTransformToCOM(lr_);
    if(vehicle_state->gear() == canbus::Chassis::GEAR_REVERSE) {
          cf_ = -control_conf_->mpc_controller_conf().cf();
          cr_ = -control_conf_->mpc_controller_conf().cr();
          for (int i = 0; i < q_param_size ; i++) {
             matrix_q_(i,i) = control_conf_->mpc_controller_conf().reverse_matrix_q(i) ;
             ADEBUG << "Q(" << i << ") " << matrix_q_(i, i) ;}
          matrix_a_(0,1) = 0.0 ;
          matrix_a_coeff_(0,2) = 1.0 ;
        }else{
          matrix_a_(0,1) = 1.0 ;
          matrix_a_coeff_(0,2) = 0.0 ;
          cf_ = control_conf_->mpc_controller_conf().cf();
          cr_ = control_conf_->mpc_controller_conf().cr();
          matrix_q_(0,0) = 0.0 ;
          matrix_q_(1,1) = 0.0 ;
	        matrix_q_(2,2) = 0.0 ;
	        matrix_q_(3,3) = 0.0 ;
          matrix_q_(4,4) = 0.0 ;
          matrix_q_(5,5) = 0.0 ;
          ADEBUG << "Q(0,0) " << matrix_q_(0, 0) ;}
  //更新A(行车和泊车侧偏刚度大小相反)
  matrix_a_(1, 2) = (cf_ + cr_) / mass_;
  matrix_a_(3, 2) = (lf_ * cf_ - lr_ * cr_) / iz_;
  matrix_a_coeff_(1, 1) = -(cf_ + cr_) / mass_;
  matrix_a_coeff_(1, 3) = (lr_ * cr_ - lf_ * cf_) / mass_;
  matrix_a_coeff_(3, 1) = (lr_ * cr_ - lf_ * cf_) / iz_;
  matrix_a_coeff_(3, 3) = -1.0 * (lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / iz_;
  //更新B
  matrix_b_(1, 0) = cf_ / mass_;
  matrix_b_(3, 0) = lf_ * cf_ / iz_;
  matrix_bd_ = matrix_b_ * ts_;
   // 更新纵向误差
  ComputeLongitudinalErrors(&trajectory_analyzer_, debug);
  // 更新航向，倒车时要+pi
  UpdateDrivingOrientation();
  // Update state
  UpdateState(debug);
  // Update A、C并作离散化
  UpdateMatrix(debug);
  // 更新前馈量
  FeedforwardUpdate(debug);
  // 此处删除了高速转向情况下对q、r权重系数的动态调整
    matrix_q_updated_ = matrix_q_;
    matrix_r_updated_ = matrix_r_;
    steer_angle_feedforwardterm_updated_ = steer_angle_feedforwardterm_;
  // Q权重矩阵
  debug->add_matrix_q_updated(matrix_q_updated_(0, 0));
  debug->add_matrix_q_updated(matrix_q_updated_(1, 1));
  debug->add_matrix_q_updated(matrix_q_updated_(2, 2));
  debug->add_matrix_q_updated(matrix_q_updated_(3, 3));
  // R权重矩阵
  debug->add_matrix_r_updated(matrix_r_updated_(0, 0));
  debug->add_matrix_r_updated(matrix_r_updated_(1, 1));
 // 计算求解MPC的约束
  Eigen::MatrixXd control_matrix(controls_, 1);
  control_matrix << 0, 0;
  Eigen::MatrixXd reference_state(basic_state_size_, 1);
  reference_state << 0, 0, 0, 0, 0, 0;
  std::vector<Eigen::MatrixXd> reference(horizon_, reference_state);
  Eigen::MatrixXd lower_bound(controls_, 1);
  lower_bound << -wheel_single_direction_max_degree_, max_deceleration_;
  Eigen::MatrixXd upper_bound(controls_, 1);
  upper_bound << wheel_single_direction_max_degree_, max_acceleration_;
  std::vector<Eigen::MatrixXd> control(horizon_, control_matrix);

  // 求解线性MPC
  double mpc_start_timestamp = Clock::NowInSeconds();
  double steer_angle_feedback = 0.0;
  //double acc_feedback = 0.0;
  double speed_feedback = 0.0;//不考虑位置环，将station_error的权重置为0
  if (common::math::SolveLinearMPC(
          matrix_ad_, matrix_bd_, matrix_cd_, matrix_q_updated_,
          matrix_r_updated_, lower_bound, upper_bound, matrix_state_, reference,
          mpc_eps_, mpc_max_iteration_, &control) != true) {
    AERROR << "MPC solver failed";
  } else {
    ADEBUG << "MPC problem solved! ";
    steer_angle_feedback = Wheel2SteerToSteerAngle(control[0](0, 0));
    speed_feedback = control[0](1, 0);
  }
  double mpc_end_timestamp = Clock::NowInSeconds();
  ADEBUG << "MPC core algorithm: calculation time is: "
         << (mpc_end_timestamp - mpc_start_timestamp) * 1000 << " ms.";
  //******************************泊车或召唤场景下开始计算横纵向控制量
  // TODO(QiL): evaluate whether need to add spline smoothing after the result
  //  计算横向控制量：方向盘转角=反馈+前馈
  steering_angle = steer_angle_feedback + steer_angle_feedforwardterm_updated_;
  ComputeSteeringAngle(steering_angle);
  cmd->set_steering_angle(steering_angle) ;
  // 此处删除了高速工况下对方向盘转角的转向约束的设计逻辑
  // 计算纵向控制量：只考虑速度环，纵向控制量=纵向速度反馈值+参考速度
  debug->set_speed_feedback_closeloop(speed_feedback);
  speed_cmd = speed_feedback + debug->speed_reference();
  }else{
 ADEBUG << "Driving Mode" ;
        //重置泊车第一端轨迹状态
        debug->set_is_parking_mode(false) ;
        debug->set_is_summon_mode(false);
        //轨迹不转化到质心
        cf_ = control_conf_->mpc_controller_conf().cf();
        cr_ = control_conf_->mpc_controller_conf().cr();
        //Q矩阵
        for (int i = 0; i < q_param_size ; i++){
          matrix_q_(i,i) = control_conf_->mpc_controller_conf().matrix_q(i) ;
          ADEBUG << "Q(" << i << ") " << matrix_q_(i, i) ;}
  //更新A(行车和泊车侧偏刚度大小相反)
  matrix_a_(1, 2) = (cf_ + cr_) / mass_;
  matrix_a_(3, 2) = (lf_ * cf_ - lr_ * cr_) / iz_;
  matrix_a_coeff_(1, 1) = -(cf_ + cr_) / mass_;
  matrix_a_coeff_(1, 3) = (lr_ * cr_ - lf_ * cf_) / mass_;
  matrix_a_coeff_(3, 1) = (lr_ * cr_ - lf_ * cf_) / iz_;
  matrix_a_coeff_(3, 3) = -1.0 * (lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / iz_;
  //更新B
  matrix_b_(1, 0) = cf_ / mass_;
  matrix_b_(3, 0) = lf_ * cf_ / iz_;
  matrix_bd_ = matrix_b_ * ts_;
  // 更新纵向误差
  ComputeLongitudinalErrors(&trajectory_analyzer_, debug);
  // 更新航向，倒车时要+pi
  UpdateDrivingOrientation();
  // Update state
  UpdateState(debug);
  // Update A、C并作离散化
  UpdateMatrix(debug);
  // 更新前馈量
  FeedforwardUpdate(debug);
  // 此处删除了高速转向情况下对q、r权重系数的动态调整
    matrix_q_updated_ = matrix_q_;
    matrix_r_updated_ = matrix_r_;
    steer_angle_feedforwardterm_updated_ = steer_angle_feedforwardterm_;
  // Q权重矩阵
  debug->add_matrix_q_updated(matrix_q_updated_(0, 0));
  debug->add_matrix_q_updated(matrix_q_updated_(1, 1));
  debug->add_matrix_q_updated(matrix_q_updated_(2, 2));
  debug->add_matrix_q_updated(matrix_q_updated_(3, 3));
  // R权重矩阵
  debug->add_matrix_r_updated(matrix_r_updated_(0, 0));
  debug->add_matrix_r_updated(matrix_r_updated_(1, 1));
 // 计算求解MPC的约束
  Eigen::MatrixXd control_matrix(controls_, 1);
  control_matrix << 0, 0;
  Eigen::MatrixXd reference_state(basic_state_size_, 1);
  reference_state << 0, 0, 0, 0, 0, 0;
  std::vector<Eigen::MatrixXd> reference(horizon_, reference_state);
  Eigen::MatrixXd lower_bound(controls_, 1);
  lower_bound << -wheel_single_direction_max_degree_, max_deceleration_;
  Eigen::MatrixXd upper_bound(controls_, 1);
  upper_bound << wheel_single_direction_max_degree_, max_acceleration_;
  std::vector<Eigen::MatrixXd> control(horizon_, control_matrix);

  // 求解线性MPC
  double mpc_start_timestamp = Clock::NowInSeconds();
  //double steer_angle_feedback = 0.0;
  //double acc_feedback = 0.0;
  double speed_feedback = 0.0;//不考虑位置环，将station_error的权重置为0
  double speed_controller_offset = 0.0;
  double speed_controller_input = 0.0;

  if (common::math::SolveLinearMPC(
          matrix_ad_, matrix_bd_, matrix_cd_, matrix_q_updated_,
          matrix_r_updated_, lower_bound, upper_bound, matrix_state_, reference,
          mpc_eps_, mpc_max_iteration_, &control) != true) {
    AERROR << "MPC solver failed";
  } else {
    ADEBUG << "MPC problem solved! ";
    steer_angle_feedback = Wheel2SteerToSteerAngle(control[0](0, 0));
    speed_feedback = control[0](1, 0);
  }

  double mpc_end_timestamp = Clock::NowInSeconds();
  ADEBUG << "MPC core algorithm: calculation time is: "
         << (mpc_end_timestamp - mpc_start_timestamp) * 1000 << " ms.";
  //******************************开始计算横纵向控制量
  // TODO(QiL): evaluate whether need to add spline smoothing after the result
  //  计算横向控制量：方向盘转角=反馈+前馈
  steering_angle = steer_angle_feedback + steer_angle_feedforwardterm_updated_;
  ComputeSteeringAngle(steering_angle);
  cmd->set_steering_angle(steering_angle) ;
  // 此处删除了高速工况下对方向盘转角的转向约束的设计逻辑
  // 计算纵向控制量：只考虑速度环，纵向控制量=纵向速度反馈值+参考速度
  debug->set_speed_feedback_closeloop(speed_feedback);
  speed_controller_input = debug->speed_error();
  speed_controller_input = common::math::Clamp(speed_controller_input, -speed_controller_input_limit, speed_controller_input_limit);

  ADEBUG << "speed_controller_input: "<< speed_controller_input;

  if (!planning_published_trajectory -> decision().main_decision().has_parking()
  && !planning_published_trajectory -> decision().main_decision().has_summon()
  && chassis_->speed_mps() > 1.7){
    double acc_k = 0.4;  //SN-2-F软件版本参数为0.4
    speed_controller_offset = speed_pid_controller_.Control(speed_controller_input, ts_) - VehicleStateProvider::instance()->linear_acceleration() * acc_k;
  }
  AINFO << "speed_controller_offset = "<< speed_controller_offset;

  speed_cmd = speed_feedback + debug->speed_reference() + speed_controller_offset;
}

  // 纵向停车逻辑，还待一一核实
  // TODO(QiL): add pitch angle feed forward to accommodate for 3D control
  debug->set_is_full_stop(false);
  GetPathRemain(debug);


  ADEBUG << "path remain 1: " << debug->path_remain();
  if( chassis->gear_location() == previous_gear && previous_stop_distance < 0.05
  && std::abs(debug -> path_remain()) < 0.15)
  {
    debug -> set_path_remain(0);
    ADEBUG << "path_remain set to 0";
  }

  ADEBUG << "path remain 2: " << debug->path_remain() ;
  ADEBUG << "speed_reference " << debug->speed_reference();

  //行车刹车逻辑改变
  //到1米时，不再用规划的距离和速度，将速度降为0.5m/s，距离每帧下降0.005


//  if(!trajectory_message_ -> decision().main_decision().has_parking() &&
//  !trajectory_message_ -> decision().main_decision().has_summon())
//  {
//    ADEBUG << "enter the driving mode";
  GetPassedDistanceByEsp(debug, chassis_);

  ADEBUG << "是否含召出模式：" << trajectory_message_ -> decision().main_decision().has_summon();
  ADEBUG << "两帧距离差值：" << previous_stop_distance - debug -> path_remain();

  if(std::abs(debug -> path_remain()) > 10)
  {
    if(debug -> path_remain() < 0)
    {
      debug -> set_path_remain(-10);
    }
    else
    {
      debug -> set_path_remain(10);
    }
  }

  if( !trajectory_message_ -> decision().main_decision().has_parking()
&& !trajectory_message_ -> decision().main_decision().has_summon() && std::abs(debug -> path_remain()) < 8)
  {
    speed_cmd = 0.8;
  }

  if(std::abs(debug -> path_remain()) < 3)
  {
    speed_cmd = 0.3;
  }

  if ( std::fabs(debug -> path_remain()) < 0.6)
  {
    ADEBUG << "enter the 60cm mode";
    ADEBUG << "driving mode, esp passed distance is:" << debug -> passed_distance_by_esp();
    speed_cmd = 0.15;

    if(trajectory_message_ -> decision().main_decision().has_parking() || trajectory_message_ -> decision().main_decision().has_summon())
    {
      ADEBUG << "enter the parking mode";
      speed_cmd = 0.15;
    }
/*      else if(trajectory_message_ -> decision().main_decision().has_summon())
    {
      ADEBUG << "enter the summon mode";
      speed_cmd = 0.3;
    }
*/
    ADEBUG << "driving mode,the previous_stop_distance is: " << previous_stop_distance;
//      if (!(std::fabs(debug -> path_remain() - previous_stop_distance) > 0.3)){
    if (previous_stop_distance != 0)
    {
      if (chassis->gear_location() == canbus::Chassis::GEAR_DRIVE)
      {
        debug->set_path_remain(previous_stop_distance - debug -> passed_distance_by_esp());

        if (debug -> path_remain() < 0)
        {
          debug -> set_path_remain(0);
        }
      }

      if (chassis->gear_location() == canbus::Chassis::GEAR_REVERSE)
      {
        debug->set_path_remain(-(previous_stop_distance - debug -> passed_distance_by_esp()));

        if (debug -> path_remain() > 0)
        {
          debug -> set_path_remain(0);
        }
      }

    }
//      debug->set_path_remain(previous_stop_distance - 0.005);
    ADEBUG << "The maintain remain_path is: " << debug -> path_remain();
//      }
  }
  else
  {
    total_passed_distance_by_esp = 0;
  }

  if( std::abs(debug -> path_remain()) < FLAGS_stop_path_remain
  || debug -> path_remain() < 0 && chassis->gear_location() == canbus::Chassis::GEAR_DRIVE
  || debug -> path_remain() > 0 && chassis->gear_location() == canbus::Chassis::GEAR_REVERSE)
  {
    speed_cmd = 0;
    debug->set_path_remain(0);
    debug->set_is_full_stop(true);
//    esp_pulse_num = 0;
//    total_passed_distance_by_esp = 0;
    ADEBUG << "2cm stop" << debug -> path_remain();
  }

  //判断是否到达终点
  //添加剩余路径判断
  if (std::fabs(debug->speed_reference())<FLAGS_max_abs_speed_when_stopped
    && std::fabs(debug -> path_remain()) < FLAGS_stop_path_remain)
  {
    debug->set_path_remain(0);
    speed_cmd = 0.0 ;
    AINFO << "Stop location reached";
    debug->set_is_full_stop(true);
//    esp_pulse_num = 0;
//    total_passed_distance_by_esp = 0;
  } else {
    AINFO << "NOT reached Stop location" ;
    debug->set_is_full_stop(false);
  }

  if (!debug->is_full_stop())
  {
    speed_cmd = common::math::Clamp(speed_cmd,0.15,2.5);
  }

  //档位控制，只有停车状态或者空档状态下才能换档
  if (chassis->gear_location() != trajectory_message_->gear()){
    ADEBUG << "Planning change gear,send stop command!";
    speed_cmd = 0;
    debug->set_path_remain(0);
  }

  ADEBUG << "car speed"<<vehicle_state->linear_velocity();

  if (std::fabs(vehicle_state->linear_velocity()) <=FLAGS_max_abs_speed_when_stopped ||
      chassis->gear_location() == trajectory_message_->gear() ||
      chassis->gear_location() == canbus::Chassis::GEAR_NEUTRAL) {
      cmd->set_gear_location(trajectory_message_->gear());
      ADEBUG <<"gear change";
  } else {
    cmd->set_gear_location(chassis->gear_location());
    ADEBUG << "keep gear" ;
  }
  ADEBUG << "planning gear " << trajectory_message_->gear() ;
  ADEBUG << "chassis gear " << chassis->gear_location();
  ADEBUG << "control gear " <<cmd->gear_location() ;
  //previous_speed_cmd = speed_cmd ;
  cmd->set_speed(speed_cmd*3.6);
  cmd->set_pam_esp_stop_distance(std::fabs(debug->path_remain()));
  AINFO << "PPPPlanning speed is :" << debug->speed_reference() ;
  AINFO << "speed command is :" << speed_cmd ;
  AINFO << "Distance command" << cmd->pam_esp_stop_distance();
  // 纵向debug信息
  debug->set_speed_real(chassis_->speed_mps());
  debug->set_speed_cmd(speed_cmd);

  cmd->set_steering_rate(FLAGS_steer_angle_rate);
  // 横向debug信息
  debug->set_heading(vehicle_state->heading());
  debug->set_steering_position(chassis->steering_percentage());
  //debug->set_steering_angle(steering_angle);
  debug->set_steer_angle_feedforward(steer_angle_feedforwardterm_updated_);
  debug->set_steer_angle_feedback(steer_angle_feedback);
  debug->set_steering_target_position_error(steering_angle - chassis->steering_percentage()) ;
  ADEBUG << "steering_position:" << debug->steering_position();
  ADEBUG << "steering_angle :" << steering_angle ;
  debug->set_steering_angle_cmd(steering_angle);

  // 处理日志文件，将debug和chassis信息写入csv文件中
  ProcessLogs(debug, chassis);
  return Status::OK();
}

/**********重置控制器状态**********/
Status MPCController::Reset() {
  previous_heading_error_ = 0.0;
  previous_lateral_error_ = 0.0;
  speed_pid_controller_.Reset();
  return Status::OK();
}

/**********加载控制标定表**********/
void MPCController::LoadControlCalibrationTable(
    const MPCControllerConf &mpc_controller_conf) {
  const auto &control_table = mpc_controller_conf.calibration_table();
  AINFO << "Control calibration table loaded";
  AINFO << "Control calibration table size is "
        << control_table.calibration_size();
  Interpolation2D::DataType xyz;
  for (const auto &calibration : control_table.calibration()) {
    xyz.push_back(std::make_tuple(calibration.speed(),
                                  calibration.acceleration(),
                                  calibration.command()));
  }
  control_interpolation_.reset(new Interpolation2D);
  CHECK(control_interpolation_->Init(xyz))
      << "Fail to load control calibration table";
}

/**********更新MPC状态量**********/
void MPCController::UpdateState(SimpleMPCDebug *debug) {
  auto vehicle_state = VehicleStateProvider::instance();
  if (debug->is_parking_mode() || debug->is_summon_mode()){
  const auto &com = vehicle_state->ComputeCOMPosition(lr_);
  vehicle_x = com.x() ;
  vehicle_y = com.y() ;
  ComputeLateralErrors(vehicle_x, vehicle_y,
                       driving_orientation,
                       vehicle_state->linear_velocity(),
                       vehicle_state->angular_velocity(),
                       trajectory_analyzer_, debug);
  }else{
  ComputeLateralErrors(vehicle_state->x(),vehicle_state->y(),
                       driving_orientation,
                       vehicle_state->linear_velocity(),
                       vehicle_state->angular_velocity(),
                       trajectory_analyzer_, debug);
  }
  // Reverse heading error if vehicle is going in reverse
    if (vehicle_state->gear() ==
        canbus::Chassis::GEAR_REVERSE) {
     debug->set_heading_error(-debug->heading_error());
      AINFO << "vehicle is going in reverse!" ;
    }
  // State matrix update;
  matrix_state_(0, 0) = debug->lateral_error();
  matrix_state_(1, 0) = debug->lateral_error_rate();
  matrix_state_(2, 0) = debug->heading_error();
  matrix_state_(3, 0) = debug->heading_error_rate();
  matrix_state_(4, 0) = debug->station_error();
  matrix_state_(5, 0) = debug->speed_error();
  ADEBUG << "lateral_error " <<debug->lateral_error();
  ADEBUG << "lateral_error_rate " <<debug->lateral_error_rate();
  ADEBUG << "heading_error " <<debug->heading_error();
  ADEBUG << "heading_error_rate " <<debug->heading_error_rate();
  ADEBUG << "station_error " <<debug->station_error();
  ADEBUG << "speed_error " <<debug->speed_error();
  AINFO << "Update state X succeed!";
}

/**********更新系数矩阵A，C并作离散化处理**********/
void MPCController::UpdateMatrix(SimpleMPCDebug *debug) {
  double v = 0.0 ;
  auto vehicle_state = VehicleStateProvider::instance();
  if (VehicleStateProvider::instance()->gear() ==
      canbus::Chassis::GEAR_REVERSE) {
    v = std::min(-VehicleStateProvider::instance()->linear_velocity(),
                 -minimum_speed_protection_);
    ADEBUG << "UpdateMatrix v= " << v ;
    matrix_a_(0, 2) = matrix_a_coeff_(0, 2) * v;
  } else{
     v = std::max(vehicle_state->linear_velocity(),
                            minimum_speed_protection_);
      matrix_a_(0, 2) = 0.0;
  }
  matrix_a_(1, 1) = matrix_a_coeff_(1, 1) / v;
  matrix_a_(1, 3) = matrix_a_coeff_(1, 3) / v;
  matrix_a_(3, 1) = matrix_a_coeff_(3, 1) / v;
  matrix_a_(3, 3) = matrix_a_coeff_(3, 3) / v;
  Matrix matrix_i = Matrix::Identity(matrix_a_.cols(), matrix_a_.cols());
  matrix_ad_ = (matrix_i - ts_ * 0.5 * matrix_a_).inverse() *
               (matrix_i + ts_ * 0.5 * matrix_a_);
  matrix_c_(1, 0) = (lr_ * cr_ - lf_ * cf_) / mass_ / v - v;
  matrix_c_(3, 0) = -(lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / iz_ / v;
  matrix_cd_ = matrix_c_ * debug->heading_error_rate() * ts_;
   AINFO << "Update Matrix succeed!" ;
}

/**********计算前馈控制量，考虑曲率**********/
void MPCController::FeedforwardUpdate(SimpleMPCDebug *debug) {
  steer_angle_feedforwardterm_ =
      Wheel2SteerToSteerAngle(wheelbase_ * debug->curvature());
}

/**********计算最后的经过滤波和限幅的方向盘转角**********/
void MPCController::ComputeSteeringAngle(double & steering_angle_)
{
  // steering_angle = feedback + feedforward ;
 //限制在方向盘最大转角内
 //  steering_angle = common::math::Clamp(steering_angle,-steering_max_degree_,steering_max_degree_);
   ADEBUG << "origin steering_angle" << steering_angle_ ;
  //数字滤波
  //steering_angle_ = digital_filter_.Filter(steering_angle);
  //ADEBUG << "after digital filter steering_angle" << steering_angle_ ;
  //和底盘反馈当前方向盘角度对比,前后差值不能超过90度
   double steering_angle_diff = steering_angle_ - chassis_->steering_percentage() ;
   steering_angle_diff = common::math::Clamp(steering_angle_diff,-60.0,60.0) ;
   steering_angle_ = chassis_->steering_percentage() + steering_angle_diff ;
   ADEBUG << "after chassis limited steering angle " << steering_angle_ ;
  //均值滤波
  // steering_angle_ = mean_filter.Update(steering_angle_);
  //限制角度防止底盘执行超调
   steering_angle = common::math::Clamp(steering_angle_,-482.0,482.0) ;
   return ;
}

/**********计算横向误差**********/
void MPCController::ComputeLateralErrors(
    const double x, const double y, const double theta, const double linear_v,
    const double angular_v, const TrajectoryAnalyzer &trajectory_analyzer,
    SimpleMPCDebug *debug) {
  // 找车辆当前点距离参考轨迹最近的匹配点
  const auto matched_point = trajectory_analyzer.QueryMatchedPathPoint(x,y);
  double match_theta = matched_point.theta() ;
  const double x_error = x - matched_point.x();
  const double y_error = y - matched_point.y();
  if (VehicleStateProvider::instance()->gear() == canbus::Chassis::GEAR_REVERSE) {
      match_theta =common::math::NormalizeAngle(matched_point.theta() + M_PI );
    }
  // 计算Frenet坐标下的横向误差、横向误差变化率和航向误差
  const double cos_matched_theta = std::cos(matched_point.theta());
  const double sin_matched_theta = std::sin(matched_point.theta());
  ADEBUG << "cos_matched_theta " << cos_matched_theta ;
  ADEBUG << "sin_matched_theta " << sin_matched_theta ;
  // d_error = cos_matched_theta * dy - sin_matched_theta * dx;
  debug->set_lateral_error(cos_matched_theta * y_error - sin_matched_theta * x_error);
  const double delta_theta =
      common::math::NormalizeAngle(theta - matched_point.theta());
  const double sin_delta_theta = std::sin(delta_theta);
  // d_error_dot = chassis_v * sin_delta_theta;
  debug->set_lateral_error_rate(linear_v * sin_delta_theta);
  // theta_error = delta_theta;
  debug->set_heading_error(delta_theta);
  debug->set_x_error(x - matched_point.x());
  debug->set_y_error(y - matched_point.y());
  ADEBUG << "x_error y_error heading_error " << x - matched_point.x() << " "
                                               << y - matched_point.y() << " "
                                               << delta_theta ;

  // 计算航向误差变化率(考虑倒车)
  // theta_error_dot = angular_v - matched_point.kappa() *matched_point.v();
   if (VehicleStateProvider::instance()->gear() == canbus::Chassis::GEAR_REVERSE){
      debug->set_heading_rate(-angular_v) ;
    }else{
      debug->set_heading_rate(angular_v);
    }
  debug->set_heading_error_rate(debug->heading_rate()- matched_point.kappa() *linear_v);
  // matched_theta = matched_point.theta();
  debug->set_ref_heading(matched_point.theta());
  // matched_kappa = matched_point.kappa();
  debug->set_curvature(matched_point.kappa());
}

/**********计算纵向误差**********/
void MPCController::ComputeLongitudinalErrors(
    const TrajectoryAnalyzer *trajectory_analyzer, SimpleMPCDebug *debug) {
  // the decomposed vehicle motion onto Frenet frame
  // s: longitudinal accumulated distance along reference trajectory
  // s_dot: longitudinal velocity along reference trajectory
  // d: lateral distance w.r.t. reference trajectory
  // d_dot: lateral distance change rate, i.e. dd/dt
  double s_matched = 0.0;
  double s_dot_matched = 0.0;
  double d_matched = 0.0;
  double d_dot_matched = 0.0;
  auto vehicle_state = VehicleStateProvider::instance();

  const auto matched_point = trajectory_analyzer->QueryMatchedPathPoint(
      vehicle_state->x(),
      vehicle_state->y());

  trajectory_analyzer->ToTrajectoryFrame(
      vehicle_state->x(),
      vehicle_state->y(),
      vehicle_state->heading(),
      vehicle_state->linear_velocity(), matched_point,
      &s_matched, &s_dot_matched, &d_matched, &d_dot_matched);

  // 根据当前的控制时间来找最近点
  const double current_control_time = Clock::NowInSeconds();
  TrajectoryPoint reference_point =
      trajectory_analyzer->QueryNearestPointByAbsoluteTime(
          current_control_time);

  ADEBUG << "matched point:" << matched_point.DebugString();
  ADEBUG << "reference point:" << reference_point.DebugString();
  // 计算纵向位置误差和速度误差
  debug->set_station_error(reference_point.path_point().s() - s_matched);
  debug->set_speed_error(reference_point.v() - s_dot_matched);

  debug->set_station_reference(reference_point.path_point().s());
  debug->set_speed_reference(reference_point.v());
  debug->set_acceleration_reference(reference_point.a());
  debug->set_current_station(s_matched);
  debug->set_vehicle_speed(
      vehicle_state->linear_velocity());
  debug->set_vehicle_acceleration(vehicle_state->linear_acceleration());
}

/**********更新车辆的行驶航向**********/
void MPCController::UpdateDrivingOrientation() {
  auto vehicle_state = VehicleStateProvider::instance();
  driving_orientation = vehicle_state->heading();
  matrix_bd_ = matrix_b_ * ts_;
  // Reverse the driving direction if the vehicle is in reverse mode
  if (FLAGS_reverse_heading_control) {
    if (vehicle_state->gear() == canbus::Chassis::GEAR_REVERSE) {
      driving_orientation =
          common::math::NormalizeAngle(driving_orientation + M_PI);
      // Update Matrix_b for reverse mode
      matrix_bd_ = -matrix_b_ * ts_;
      ADEBUG << "Matrix_b changed due to gear direction";
    }
  }
}

/**********计算余留的路径**********/
void MPCController::GetPathRemain(SimpleMPCDebug *debug) {
  int stop_index = trajectory_message_->trajectory_point_size() - 1; //---------------轨迹最后一个点下标;
  debug->set_path_remain(
      trajectory_message_->trajectory_point(stop_index).path_point().s() - debug->current_station());
  ADEBUG << "stop index " << stop_index ;
  ADEBUG << "trajectory point size" << trajectory_message_->trajectory_point_size() ;
  ADEBUG << "last trajectory point station " << trajectory_message_->trajectory_point(stop_index).path_point().s() ;
  return ;
}

  void MPCController::GetPassedDistanceByEsp(SimpleMPCDebug *debug, const canbus::Chassis *chassis)
  {
    if(chassis -> driving_mode() != jmc_auto::canbus::Chassis::COMPLETE_AUTO_DRIVE)
    {
      total_passed_distance_by_esp = 0;
//      esp_pulse_num = 0;
      return;
    }

    if((chassis->gear_location() != previous_gear))
    {
//      esp_pulse_num = 0;
      ADEBUG << "前后挡位不相等，上一帧挡位：" << previous_gear;
      ADEBUG << "前后挡位不相等，当前挡位：" << chassis->gear_location();
      total_passed_distance_by_esp = 0;
    }
  //利用esp四个轮速脉冲计算车辆行驶的距离
    double esp_wheelPulse_fl = chassis -> esp_wheelpulse_fl();
    double esp_wheelPulse_fr = chassis -> esp_wheelpulse_fr();
    double esp_wheelPulse_rl = chassis -> esp_wheelpulse_rl();
    double esp_wheelPulse_rr = chassis -> esp_wheelpulse_rr();
    double wheel_rolling_radius = vehicle_param_.wheel_rolling_radius();
    double inc_esp_wheelPulse_fl = 0.0;
    double inc_esp_wheelPulse_fr = 0.0;
    double inc_esp_wheelPulse_rl = 0.0;
    double inc_esp_wheelPulse_rr = 0.0;
    double per_passed_distance_by_esp = 0.0;

    ADEBUG << "车轮滚动半径为：" << wheel_rolling_radius;

  //轮速脉冲值最大为255
    if(std::abs(esp_wheelPulse_fl - previous_esp_wheelPulse_fl) > 100)
    {
      inc_esp_wheelPulse_fl = 255 - std::abs(esp_wheelPulse_fl - previous_esp_wheelPulse_fl);
    }
    else
    {
      inc_esp_wheelPulse_fl = std::abs(esp_wheelPulse_fl - previous_esp_wheelPulse_fl);
    }

    if(std::abs(esp_wheelPulse_fr - previous_esp_wheelPulse_fr) > 100)
    {
      inc_esp_wheelPulse_fr = 255 - std::abs(esp_wheelPulse_fr - previous_esp_wheelPulse_fr);
    }
    else
    {
      inc_esp_wheelPulse_fr = std::abs(esp_wheelPulse_fr - previous_esp_wheelPulse_fr);
    }

    if(std::abs(esp_wheelPulse_rl - previous_esp_wheelPulse_rl) > 100)
    {
      inc_esp_wheelPulse_rl = 255 - std::abs(esp_wheelPulse_rl - previous_esp_wheelPulse_rl);
    }
    else
    {
      inc_esp_wheelPulse_rl = std::abs(esp_wheelPulse_rl - previous_esp_wheelPulse_rl);
    }

    if(std::abs(esp_wheelPulse_rr - previous_esp_wheelPulse_rr) > 100)
    {
      inc_esp_wheelPulse_rr = 255 - std::abs(esp_wheelPulse_rr - previous_esp_wheelPulse_rr);
    }
    else
    {
      inc_esp_wheelPulse_rr = std::abs(esp_wheelPulse_rr - previous_esp_wheelPulse_rr);
    }

    double inc_esp_wheelPulse = (inc_esp_wheelPulse_fl + inc_esp_wheelPulse_fr + inc_esp_wheelPulse_rl +
    inc_esp_wheelPulse_rr) / 4;

    if( inc_esp_wheelPulse > 2)
    {
      //ADEBUG << "esp_pulse_num is: " << esp_pulse_num;
      ADEBUG << "previous_esp_wheelPulse_fl脉冲值为：" << previous_esp_wheelPulse_fl;
      ADEBUG << "previous_esp_wheelPulse_fr脉冲值为：" << previous_esp_wheelPulse_fr;
      ADEBUG << "previous_esp_wheelPulse_rl脉冲值为：" << previous_esp_wheelPulse_rl;
      ADEBUG << "previous_esp_wheelPulse_rr脉冲值为：" << previous_esp_wheelPulse_rr;
      ADEBUG << "esp_wheelPulse_fl脉冲值为：" << esp_wheelPulse_fl;
      ADEBUG << "esp_wheelPulse_fr脉冲值为：" << esp_wheelPulse_fr;
      ADEBUG << "esp_wheelPulse_rl脉冲值为：" << esp_wheelPulse_rl;
      ADEBUG << "esp_wheelPulse_rr脉冲值为：" << esp_wheelPulse_rr;
      ADEBUG << " inc_esp_wheelPulse > 2,is :" <<  inc_esp_wheelPulse;
      inc_esp_wheelPulse = 0;
    }

    per_passed_distance_by_esp = 2 * M_PI * wheel_rolling_radius * inc_esp_wheelPulse / 48 * 0.472 / 0.35 * 0.9;

    /*
    if (esp_pulse_num == 0)
    {
        per_passed_distance_by_esp = 0;
    }
    */

    total_passed_distance_by_esp = total_passed_distance_by_esp + per_passed_distance_by_esp;  //单位为m

    ADEBUG << "111total_passed_distance_by_esp为：" << total_passed_distance_by_esp;

  //  if (esp_pulse_num > 1){
  //    esp_pulse_num = 1;
  //  }
    debug -> set_passed_distance_by_esp(per_passed_distance_by_esp);


    previous_esp_wheelPulse_fl = esp_wheelPulse_fl;
    previous_esp_wheelPulse_fr = esp_wheelPulse_fr;
    previous_esp_wheelPulse_rl = esp_wheelPulse_rl;
    previous_esp_wheelPulse_rr = esp_wheelPulse_rr;

    ADEBUG << "均值脉冲值为：" << inc_esp_wheelPulse;
    ADEBUG << "per_passed_distance_by_esp为：" << debug -> passed_distance_by_esp();





    previous_gear = chassis->gear_location();
  //  esp_pulse_num ++;
  //  ADEBUG << "esp_pulse_num is:" << esp_pulse_num;
  }

}  // namespace control
}  // namespace jmc_auto
