/****************************************************************************
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
#include "modules/control/controller/lon_controller.h"

#include <cstdio>
#include <utility>

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/log.h"
#include "modules/common/math/math_utils.h"
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

constexpr double GRA_ACC = 9.8;

LonController::LonController() : name_(ControlConf_ControllerType_Name(ControlConf::LON_CONTROLLER)) {
  if (FLAGS_enable_csv_debug) {
    time_t rawtime;
    char name_buffer[80];
    std::time(&rawtime);
    strftime(name_buffer, 80, "data/csv/speed_log__%F_%H%M%S.csv", localtime(&rawtime));
    speed_log_file_ = fopen(name_buffer, "w");
    if (speed_log_file_ == nullptr) {
      AERROR << "Fail to open file:" << name_buffer;
      FLAGS_enable_csv_debug = false;
    }
    if (speed_log_file_ != nullptr) {
      fprintf(speed_log_file_,
        "timestramp,"
        "station_reference,"
        "current_station,"
        "station_error,"
//       "station_error_limited,"
//        "preview_station_error,"
        "speed_reference,"
        "speed_real,"
        "speed_cmd,"
        "speed_error,"
        "speed_error_limited,"
        "preview_speed_reference,"
        "preview_speed_error,"
        "preview_acceleration_reference,"
        "match_acceleration_reference,"
//        "speed_offset,"
        "speed_controller_offset,"
        "path_remain,"
        "match_x,"
        "match_y,"
        "match_speed,"
        "match_acceleration,"
        "vehicle_x,"
        "vehicle_y,"
        "vehicle_speed,"
        "vehicle_acceleration,"
        "is_full_stop,"
        "wheel_pulse,"
        "\r\n");
      fflush(speed_log_file_);
    }
    AINFO << name_ << " used.";
  }
}

void LonController::CloseLogFile()
{
  if (FLAGS_enable_csv_debug)
  {
    if (speed_log_file_ != nullptr)
    {
      fclose(speed_log_file_);
      speed_log_file_ = nullptr;
    }
  }
}
void LonController::Stop() { CloseLogFile(); }

LonController::~LonController() { CloseLogFile(); }

Status LonController::Init(const ControlConf *control_conf)
{
  control_conf_ = control_conf;
  if (control_conf_ == nullptr)
  {
    controller_initialized_ = false;
    AERROR << "get_longitudinal_param() nullptr";
    return Status(ErrorCode::CONTROL_INIT_ERROR,
                  "Failed to load LonController conf");
  }
  const LonControllerConf &lon_controller_conf = control_conf_->lon_controller_conf();

//  station_pid_controller_.Init(lon_controller_conf.station_pid_conf());
  speed_pid_controller_.Init(lon_controller_conf.low_speed_pid_conf());
  vehicle_param_.CopyFrom(common::VehicleConfigHelper::instance()->GetConfig().vehicle_param());

//暂未用到均值滤波器
  //SetDigitalFilterPitchAngle(lon_controller_conf);
  //lon_acc_filter_ = common::MeanFilter(
	//lon_controller_conf.mean_filter_window_size()) ;
  //LoadControlCalibrationTable(lon_controller_conf);
  controller_initialized_ = true;
  return Status::OK();
}


/*
void LonController::SetDigitalFilterPitchAngle(
    const LonControllerConf &lon_controller_conf)
{
  double cutoff_freq =
      lon_controller_conf.pitch_angle_filter_conf().cutoff_freq();
  double ts = lon_controller_conf.ts();
  SetDigitalFilter(ts, cutoff_freq, &digital_filter_pitch_angle_);
}
*/

/*
void LonController::SetDigitalFilter(double ts, double cutoff_freq, common::DigitalFilter *digital_filter) {
  std::vector<double> denominators;
  std::vector<double> numerators;
  common::LpfCoefficients(ts, cutoff_freq, &denominators, &numerators);
  digital_filter->set_coefficients(denominators, numerators);
}
*/

//用到的APA接口不再需要通过插值表求取油门值
/*
void LonController::LoadControlCalibrationTable(
    const LonControllerConf &lon_controller_conf)
{
  const auto &control_table = lon_controller_conf.calibration_table();
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
*/

Status LonController::ComputeControlCommand(
    const localization::LocalizationEstimate *localization,
    const canbus::Chassis *chassis,
    const planning::ADCTrajectory *planning_published_trajectory,
    control::ControlCommand *cmd) {
  localization_ = localization;
  chassis_ = chassis;
  trajectory_message_ = planning_published_trajectory;

  if (trajectory_analyzer_ == nullptr ||
      trajectory_analyzer_->seq_num() != trajectory_message_->header().sequence_num()) {
    trajectory_analyzer_.reset(new TrajectoryAnalyzer(trajectory_message_));
  }

  const LonControllerConf &lon_controller_conf = control_conf_->lon_controller_conf();

  auto debug = cmd->mutable_debug()->mutable_simple_lon_debug();
  debug->Clear();

  double ts = lon_controller_conf.ts();
  double preview_time = lon_controller_conf.preview_window() * ts;

  if (preview_time < 0.0) {
    const auto error_msg = common::util::StrCat(
        "Preview time set as: ", preview_time, " less than 0");
    AERROR << error_msg;
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, error_msg);
  }


  // 计算纵向误差
  ComputeLongitudinalErrors(trajectory_analyzer_.get(), preview_time, debug);

  double speed_controller_input = 0.0;
  double speed_controller_offset = 0.0 ;
  double speed_controller_input_limit = lon_controller_conf.speed_controller_input_limit();
  double speed_cmd = 0.0;

//  使用TTE的APA接口，取消了位置控制器的使用
//  double station_error_limit = lon_controller_conf.station_error_limit();
//  double station_error_limited = 0.0;
/*
  if (FLAGS_enable_speed_station_preview) {
    station_error_limited =
        common::math::Clamp(debug->preview_station_error(),
                            -station_error_limit, station_error_limit);
  } else {
    station_error_limited = common::math::Clamp(
        debug->station_error(), -station_error_limit, station_error_limit);
  }
*/

  if (trajectory_message_->gear() == canbus::Chassis::GEAR_REVERSE) {
//      station_pid_controller_.SetPID(lon_controller_conf.reverse_station_pid_conf());
      speed_pid_controller_.SetPID(lon_controller_conf.reverse_speed_pid_conf());
  }
//  double speed_offset = station_pid_controller_.Control(station_error_limited, ts);
//  ADEBUG << "Station PID speed_offset = " << speed_offset ;

/*
  if(trajectory_message_->gear() == canbus::Chassis::GEAR_REVERSE){
     speed_offset = 0.0 ;
  }
*/

  if (FLAGS_enable_speed_station_preview ) {
    speed_controller_input = debug->preview_speed_error();
  } else {
    speed_controller_input = debug->speed_error();
  }
  speed_controller_input = common::math::Clamp(speed_controller_input, -speed_controller_input_limit, speed_controller_input_limit);
  ADEBUG << "speed_controller_input: "<< speed_controller_input;

  if (!trajectory_message_ -> decision().main_decision().has_parking()
  && !trajectory_message_ -> decision().main_decision().has_summon()
  && chassis->speed_mps() > 1.7){
    double acc_k = 0.4;  //SN-2-F软件版本参数为0.4
    speed_controller_offset = speed_pid_controller_.Control(speed_controller_input, ts) - VehicleStateProvider::instance()->linear_acceleration() * acc_k;
  }
  AINFO << "speed_controller_offset = "<< speed_controller_offset;

  if (FLAGS_enable_speed_station_preview){
    speed_cmd = std::fabs(debug->preview_speed_reference() + speed_controller_offset);
  } else{
    speed_cmd = std::fabs( debug->speed_reference() + speed_controller_offset );
  }

  debug->set_is_full_stop(false);
  GetPathRemain(debug);
  ADEBUG << "path remain " << debug->path_remain() ;
  ADEBUG << "preview speed "<< debug->preview_speed_reference();
  ADEBUG << "speed_reference " << debug->speed_reference();

  //行车刹车逻辑改变
  //到1米时，不再用规划的距离和速度，将速度降为0.5m/s，距离每帧下降0.005


//  if(!trajectory_message_ -> decision().main_decision().has_parking() &&
//  !trajectory_message_ -> decision().main_decision().has_summon())
//  {
//    ADEBUG << "enter the driving mode";
    if ( std::fabs(debug -> path_remain()) < 0.6)
    {
      ADEBUG << "enter the 60cm mode";
      GetPassedDistanceByEsp(debug, chassis_);
      ADEBUG << "driving mode, esp passed distance is:" << debug -> passed_distance_by_esp();
      speed_cmd = 0.4;

      if(trajectory_message_ -> decision().main_decision().has_parking() ||
      trajectory_message_ -> decision().main_decision().has_summon())
      {
        ADEBUG << "enter the parking mode";
        speed_cmd = 0.5;
      }

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
//  }

  //泊车刹车逻辑改变，泊车采用固定速度，行车距离用ESP轮速脉冲传感器计算
  /*if(trajectory_message_ -> decision().main_decision().has_parking())
  {
    ADEBUG << "enter the parking mode";
    GetPassedDistanceByEsp(debug, chassis_);
    ADEBUG << "parking mode, esp passed distance is:" << debug -> passed_distance_by_esp();
    speed_cmd = 0.5;
    ADEBUG << "parking mode, the previous_stop_distance is: " << previous_stop_distance;
    if(previous_stop_distance != 0)
    {
      debug->set_path_remain(previous_stop_distance - debug -> passed_distance_by_esp());
      if (debug -> path_remain() < 0)
      {
        debug -> set_path_remain(0);
      }
    }
    ADEBUG << "parking mode, The maintain remain_path is: " << debug -> path_remain();
  }
  */

  if( std::abs(debug -> path_remain()) < FLAGS_stop_path_remain
  || debug -> path_remain() < 0 && chassis->gear_location() == canbus::Chassis::GEAR_DRIVE
  || debug -> path_remain() > 0 && chassis->gear_location() == canbus::Chassis::GEAR_REVERSE)
  {
    speed_cmd = 0;
    debug->set_path_remain(0);
    debug->set_is_full_stop(true);
    esp_pulse_num = 0;
    total_passed_distance_by_esp = 0;
    ADEBUG << "2cm stop" << debug -> path_remain();
  }

  //判断是否到达终点
  //添加剩余路径判断
  if (std::fabs(debug->speed_reference())<FLAGS_max_abs_speed_when_stopped
    && std::fabs(debug->preview_speed_reference() <= FLAGS_max_abs_speed_when_stopped)
    && std::fabs(debug -> path_remain()) < FLAGS_stop_path_remain)
  {
    debug->set_path_remain(0);
    speed_cmd = 0.0 ;
    AINFO << "Stop location reached";
    debug->set_is_full_stop(true);
    esp_pulse_num = 0;
    total_passed_distance_by_esp = 0;
  } else {
    AINFO << "NOT reached Stop location" ;
    debug->set_is_full_stop(false);
  }

  if (!debug->is_full_stop())
  {
    speed_cmd = common::math::Clamp(speed_cmd,0.25,2.5);
  }
 /* if (trajectory_message_->gear() == canbus::Chassis::GEAR_REVERSE && (!debug->is_full_stop())) {
    if(chassis->speed_mps() == 0)
      {
        speed_cmd = common::math::Clamp(speed_cmd,0.15,2.5);
      } else {
        speed_cmd = common::math::Clamp(speed_cmd,0.1,3.0);
      }
  } else if(trajectory_message_->gear() == canbus::Chassis::GEAR_DRIVE &&(!debug->is_full_stop())){
    if(chassis->speed_mps() == 0)
      {
        speed_cmd = common::math::Clamp(speed_cmd,0.15,3.0);
      } else {
        speed_cmd = common::math::Clamp(speed_cmd,0.1,3.0);
      }
  }
*/
//档位控制，只有停车状态或者空档状态下才能换档
  if (chassis->gear_location() != trajectory_message_->gear()){
    ADEBUG << "Planning change gear,send stop command!";
    speed_cmd = 0;
    debug->set_path_remain(0);
  }
  ADEBUG << "car speed"<<VehicleStateProvider::instance()->linear_velocity();

  if (std::fabs(VehicleStateProvider::instance()->linear_velocity()) <= FLAGS_max_abs_speed_when_stopped
    || chassis->gear_location() == trajectory_message_->gear()
    || chassis->gear_location() == canbus::Chassis::GEAR_NEUTRAL) {
    cmd->set_gear_location(trajectory_message_->gear());
    ADEBUG <<"gear change";
  } else {
    cmd->set_gear_location(chassis->gear_location());
    ADEBUG << "keep gear";
  }
  ADEBUG << "planning gear " << trajectory_message_->gear();
  ADEBUG << "chassis gear " << chassis->gear_location();
  ADEBUG << "control gear " << cmd->gear_location();
  //previous_speed_cmd = speed_cmd;
  if (!(std::fabs(debug->path_remain()) > 0.65
  && std::fabs(debug -> path_remain()) > FLAGS_stop_path_remain))
  {
    ADEBUG << "driving mode, previous_stop_distance is ava";
    previous_stop_distance = std::abs(debug->path_remain());
    ADEBUG << "path_remain < 0.65 or parking mode, previous_stop_distance is: " << previous_stop_distance;
  }
  else
  {
    previous_stop_distance = 0;
  }

  //APA接口接收目标速度指令单位为km/h，目标距离指令为cm
  //发送目标速度和目标距离的指令在control.cc文件
  cmd->set_speed(speed_cmd * 3.6);
  cmd->set_pam_esp_stop_distance( std::fabs(debug->path_remain()) * 100 );
  AINFO << "PPPPlanning speed is :" << debug->speed_reference();
  AINFO << "speed command is :" << speed_cmd;
  AINFO << "Distance command" << cmd->pam_esp_stop_distance();
  debug->set_speed_real(chassis_->speed_mps());

//output speedData
  double current_control_time = Clock::NowInSeconds();
  if (FLAGS_enable_csv_debug && speed_log_file_ != nullptr) {
    fprintf(speed_log_file_,
      "%.6f,%.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f,"
      "%.6f, %.6f, %.6f, %.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%.6f, \r\n",
      current_control_time,
      debug->station_reference(),
      debug->current_station(),
      debug->station_error(),
//      station_error_limited,
//      debug->preview_station_error(),
      debug->speed_reference(),
      debug->speed_real(),
      speed_cmd,
      debug->speed_error(),
      speed_controller_input,
      debug->preview_speed_reference(),
      debug->preview_speed_error(),
      debug->preview_acceleration_reference(),
      debug->match_acceleration_reference(),
//      speed_offset,
      speed_controller_offset,
      debug -> path_remain(),
      debug->match_x(),
      debug->match_y(),
      debug->match_speed(),
      debug->match_acceleration(),
      debug->vehicle_x(),
      debug->vehicle_y(),
      debug->vehicle_speed(),
      debug->vehicle_acceleration(),
      debug->is_full_stop(),
      debug -> passed_distance_by_esp());
  }
  return Status::OK();
}

Status LonController::Reset() {
  speed_pid_controller_.Reset();
  station_pid_controller_.Reset();//
  return Status::OK();
}

std::string LonController::Name() const { return name_; }

void LonController::ComputeLongitudinalErrors( const TrajectoryAnalyzer *trajectory_analyzer, const double preview_time, SimpleLongitudinalDebug *debug) {
  // the decomposed vehicle motion onto Frenet frame
  // s: longitudinal accumulated distance along reference trajectory
  // s_dot: longitudinal velocity along reference trajectory
  // d: lateral distance w.r.t. reference trajectory
  // d_dot: lateral distance change rate, i.e. dd/dt
  double s_matched = 0.0;
  double s_dot_matched = 0.0;
  double d_matched = 0.0;
  double d_dot_matched = 0.0;

  auto matched_point = trajectory_analyzer->QueryMatchedPathPoint(
      VehicleStateProvider::instance()->x(),
      VehicleStateProvider::instance()->y());

  trajectory_analyzer->ToTrajectoryFrame(
      VehicleStateProvider::instance()->x(),
      VehicleStateProvider::instance()->y(),
      VehicleStateProvider::instance()->heading(),
      VehicleStateProvider::instance()->linear_velocity(), matched_point,
      &s_matched, &s_dot_matched, &d_matched, &d_dot_matched);

 // double current_control_time = Clock::NowInSeconds();
//  double preview_control_time = current_control_time + preview_time;

//  TrajectoryPoint reference_point =
//      trajectory_analyzer->QueryNearestPointByAbsoluteTime(
//          current_control_time);
  TrajectoryPoint reference_point =trajectory_analyzer->QueryNearestPointByPosition( VehicleStateProvider::instance()->x(), VehicleStateProvider::instance()->y());
  double preview_control_time = reference_point.relative_time()+preview_time;
  TrajectoryPoint preview_point = trajectory_analyzer->QueryNearestPointByRelativeTime(preview_control_time);

  ADEBUG << "matched point:" << matched_point.DebugString();
  ADEBUG << "reference point:" << reference_point.DebugString();
  ADEBUG << "preview point:" << preview_point.DebugString();
  debug->set_station_error(reference_point.path_point().s() - s_matched);
//  debug->set_station_error(0);
  debug->set_speed_error(reference_point.v() - s_dot_matched);
  debug->set_station_reference(reference_point.path_point().s());
  debug->set_speed_reference(reference_point.v());
  debug->set_preview_station_error(preview_point.path_point().s() - s_matched);
//  debug->set_preview_station_error(0) ;
  debug->set_preview_speed_error(preview_point.v() - s_dot_matched);
  debug->set_preview_speed_reference(preview_point.v());
  debug->set_preview_acceleration_reference(preview_point.a());
  debug->set_current_station(s_matched);
  debug->set_preview_kappa(preview_point.path_point().kappa());
  debug->set_match_acceleration_reference(reference_point.a());
  debug->set_match_x(reference_point.path_point().x());
  debug->set_match_y(reference_point.path_point().y());
  debug->set_match_speed(std::fabs(reference_point.v()));
  debug->set_match_acceleration(reference_point.a());
  debug->set_vehicle_x(VehicleStateProvider::instance()->x());
  debug->set_vehicle_y(VehicleStateProvider::instance()->y());
  debug->set_vehicle_speed(VehicleStateProvider::instance()->linear_velocity());
  debug->set_vehicle_acceleration(VehicleStateProvider::instance()->linear_acceleration());
}

  void LonController::GetPathRemain(SimpleLongitudinalDebug *debug) {
    int stop_index = trajectory_message_->trajectory_point_size() - 1; //---------------轨迹最后一个点下标;
    debug->set_path_remain( trajectory_message_->trajectory_point(stop_index).path_point().s() - debug->current_station());
    ADEBUG << "stop index " << stop_index ;
    ADEBUG << "trajectory point size" << trajectory_message_->trajectory_point_size() ;
    ADEBUG << "last trajectory point station " << trajectory_message_->trajectory_point(stop_index).path_point().s() ;
    return ;
  }

  void LonController::GetPassedDistanceByEsp(SimpleLongitudinalDebug *debug, const canbus::Chassis *chassis)
  {
    if(chassis -> driving_mode() != jmc_auto::canbus::Chassis::COMPLETE_AUTO_DRIVE)
    {
      total_passed_distance_by_esp = 0;
      esp_pulse_num = 0;
      return;
    }

    if((chassis->gear_location() != previous_gear))
    {
      esp_pulse_num = 0;
      ADEBUG << "前后挡位不相等" << chassis->gear_location();
      ADEBUG << "前后挡位不相等" << previous_gear;
      total_passed_distance_by_esp = 0;
    }
  //利用esp四个轮速脉冲计算车辆行驶的距离
    double esp_wheelPulse_fl = chassis -> esp_wheelpulse_fl();
    double esp_wheelPulse_fr = chassis -> esp_wheelpulse_fr();
    double esp_wheelPulse_rl = chassis -> esp_wheelpulse_rl();
    double esp_wheelPulse_rr = chassis -> esp_wheelpulse_rr();
    const auto &vehicle_param_ = common::VehicleConfigHelper::instance()->GetConfig().vehicle_param();
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
      inc_esp_wheelPulse = 0;
      ADEBUG << "esp_pulse_num is: " << esp_pulse_num;
      ADEBUG << "previous_esp_wheelPulse_fl脉冲值为：" << previous_esp_wheelPulse_fl;
      ADEBUG << "previous_esp_wheelPulse_fr脉冲值为：" << previous_esp_wheelPulse_fr;
      ADEBUG << "previous_esp_wheelPulse_rl脉冲值为：" << previous_esp_wheelPulse_rl;
      ADEBUG << "previous_esp_wheelPulse_rr脉冲值为：" << previous_esp_wheelPulse_rr;
      ADEBUG << "esp_wheelPulse_fl脉冲值为：" << esp_wheelPulse_fl;
      ADEBUG << "esp_wheelPulse_fr脉冲值为：" << esp_wheelPulse_fr;
      ADEBUG << "esp_wheelPulse_rl脉冲值为：" << esp_wheelPulse_rl;
      ADEBUG << "esp_wheelPulse_rr脉冲值为：" << esp_wheelPulse_rr;
    }

    ADEBUG << "111total_passed_distance_by_esp为：" << total_passed_distance_by_esp;
    per_passed_distance_by_esp = 2 * M_PI * wheel_rolling_radius * inc_esp_wheelPulse / 48;
    if (esp_pulse_num == 0)
    {
        per_passed_distance_by_esp = 0;
    }
    else
    {
      total_passed_distance_by_esp = total_passed_distance_by_esp + per_passed_distance_by_esp;  //单位为m
    }

    if (esp_pulse_num > 1){
      esp_pulse_num = 1;
    }
    debug -> set_passed_distance_by_esp(per_passed_distance_by_esp);

    previous_esp_wheelPulse_fl = esp_wheelPulse_fl;
    previous_esp_wheelPulse_fr = esp_wheelPulse_fr;
    previous_esp_wheelPulse_rl = esp_wheelPulse_rl;
    previous_esp_wheelPulse_rr = esp_wheelPulse_rr;

    ADEBUG << "均值脉冲值为：" << inc_esp_wheelPulse;
    ADEBUG << "per_passed_distance_by_esp为：" << debug -> passed_distance_by_esp();





    previous_gear = chassis->gear_location();
    esp_pulse_num ++;
    ADEBUG << "esp_pulse_num is:" << esp_pulse_num;

  }

}  // namespace control
}  // namespace jmc_auto
