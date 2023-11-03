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


#include <cstdio>
#include <utility>

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/log.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/time/time.h"
#include "modules/common/util/string_util.h"
#include "modules/remotecontrol/common/remotecontrol_gflags.h"
#include "modules/remotecontrol/common/speed_control.h"
#include "modules/common/util/file.h"

namespace jmc_auto {
namespace remotecontrol {

using jmc_auto::common::ErrorCode;
using jmc_auto::common::Status;
using jmc_auto::common::VehicleStateProvider;
using jmc_auto::common::time::Clock;
using jmc_auto::control::LonControllerConf;

constexpr double GRA_ACC = 9.8;

SpeedControl::SpeedControl() {

}

void SpeedControl::CloseLogFile() {
}
void SpeedControl::Stop() { CloseLogFile(); }

SpeedControl::~SpeedControl() { CloseLogFile(); }

Status SpeedControl::Init() {
  CHECK(common::util::GetProtoFromFile(FLAGS_speed_control_conf_file, &control_conf_))
      << "Unable to load control conf file: " + FLAGS_speed_control_conf_file;     
  lon_controller_conf= control_conf_.lon_controller_conf();
  speed_pid_controller_.Init(lon_controller_conf.low_speed_pid_conf());

  vehicle_param_.CopyFrom(
      common::VehicleConfigHelper::instance()->GetConfig().vehicle_param());

  LoadControlCalibrationTable(lon_controller_conf);
  controller_initialized_ = true;

  return Status::OK();
}

void SpeedControl::SetDigitalFilterPitchAngle(
    const LonControllerConf &lon_controller_conf) {
}

void SpeedControl::LoadControlCalibrationTable(
    const LonControllerConf &lon_controller_conf) {
  const auto &control_table = lon_controller_conf.calibration_table();
  AINFO << "Control calibration table loaded";
  AINFO << "Control calibration table size is "
        << control_table.calibration_size();
  control::Interpolation2D::DataType xyz;
  for (const auto &calibration : control_table.calibration()) {
    xyz.push_back(std::make_tuple(calibration.speed(),
                                  calibration.acceleration(),
                                  calibration.command()));
  }
  control_interpolation_.reset(new control::Interpolation2D);
  CHECK(control_interpolation_->Init(xyz))
      << "Fail to load control calibration table";
}

Status SpeedControl::ComputeControlCommand(
    const float Target_vehicle_speed,
    const canbus::Chassis *chassis,
    jmc_auto::remote::RemoteControl *cmd) {

  if (cmd->has_pedal_brake_percent()&&cmd->pedal_brake_percent()!=0)
  {
    return Status::OK();
  }
  
  if (!control_interpolation_) {
    AERROR << "Fail to initialize calibration table.";
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR,
                  "Fail to initialize calibration table.");
  }

  
  double brake_cmd = 0.0;
  double throttle_cmd = 0.0;
  double ts = lon_controller_conf.ts();
  double speed_controller_input = 0.0;
  double speed_controller_input_limit =
      lon_controller_conf.speed_controller_input_limit();
  double speed_controller_input_limited = 0.0;

  speed_controller_input=Target_vehicle_speed-chassis->speed_mps();
 AINFO<<"speed_controller_input:"<<speed_controller_input<<"Target_vehicle_speed-chassis->speed_mps():"<<chassis->speed_mps();
  speed_controller_input_limited =
      common::math::Clamp(speed_controller_input, -speed_controller_input_limit,
                          speed_controller_input_limit);
  double acceleration_cmd_closeloop = 0.0;
 AINFO<<"speed_controller_input_limited:"<<speed_controller_input_limited;
    speed_pid_controller_.SetPID(lon_controller_conf.low_speed_pid_conf());
    acceleration_cmd_closeloop =
        speed_pid_controller_.Control(speed_controller_input_limited, ts);
   AINFO<<"acceleration_cmd_closeloop:"<<acceleration_cmd_closeloop;
  double acceleration_cmd =
      acceleration_cmd_closeloop;
 
  // if (std::fabs(VehicleStateProvider::instance()->linear_acceleration()) <=
  //          FLAGS_max_acceleration_when_stopped &&
  //      std::fabs(Target_vehicle_speed) <=
  //          vehicle_param_.max_abs_speed_when_stopped()) {
  //   acceleration_cmd = lon_controller_conf.standstill_acceleration();
  //   AINFO << "Stop location reached";

  // }

  double throttle_deadzone = lon_controller_conf.throttle_deadzone();
  double brake_deadzone = lon_controller_conf.brake_deadzone();
  AINFO<<"chassis->speed_mps()"<<chassis->speed_mps();
  double speed=static_cast<double>(chassis->speed_mps());
  AINFO<<"thrott"<<chassis->throttle_percentage();
  double thrott=(double)chassis->throttle_percentage();
  AINFO<<"speed:"<<speed<<" thrott:"<<thrott<<" gear_location"<<chassis->gear_location();
  AINFO<<"acceleration_cmd:"<<acceleration_cmd;
  double calibration_value = 0.0;
  if (!isnan(speed)&&!isnan(acceleration_cmd))
  {
    AINFO<<"start Interpolate calibration";
    calibration_value = control_interpolation_->Interpolate(
        std::make_pair(speed, acceleration_cmd));
    AINFO<<"calibration_value:"<<calibration_value;   
  }

  if (chassis->speed_mps()<= Target_vehicle_speed)
  {
    if (calibration_value >= 0) {
        throttle_cmd =calibration_value > throttle_deadzone ? calibration_value
                                                          : throttle_deadzone;
        brake_cmd = 0.0;
      }
      else
      {
        throttle_cmd = throttle_deadzone ;
        brake_cmd = 0.0;
      }
        
    }
    else if (chassis->speed_mps() > Target_vehicle_speed )
    {
          if (calibration_value >= 0) {
           throttle_cmd = throttle_deadzone-1 ;
            brake_cmd = 0.0;
        }
        else
        {
        double DecelerationFactor=1-fabs(acceleration_cmd)/0.13;
     DecelerationFactor = DecelerationFactor < 0 ? 0: DecelerationFactor; 
     throttle_cmd =DecelerationFactor*throttle_deadzone ;
     brake_cmd = 0.0;     
           throttle_cmd = throttle_deadzone-1 ;
            brake_cmd = 0.0;
        }
    }
    
    if (Target_vehicle_speed==0)
    {
     throttle_cmd =0.0 ;
     brake_cmd = 0.0;    
    }

 AINFO<<"calibration_value:"<<calibration_value<<"/r/n throttle_cmd："<<throttle_cmd<<"/r/n brake_cmd："<<brake_cmd<<"speed:"<<chassis->speed_mps();
  cmd->set_pedal_throttle_percent((float)throttle_cmd);
  cmd->set_pedal_brake_percent((float)brake_cmd);
  return Status::OK();
}

Status SpeedControl::Reset() {
  speed_pid_controller_.Reset();
  return Status::OK();
}

std::string SpeedControl::Name() const { return name_; }


void SpeedControl::SetDigitalFilter(double ts, double cutoff_freq,
                                     common::DigitalFilter *digital_filter) {
}

}  // namespace remotecontrol
}  // namespace jmc_auto
