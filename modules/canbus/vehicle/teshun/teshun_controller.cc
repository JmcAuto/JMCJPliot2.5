/* Copyright 2017 The JmcAuto Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/

#include "modules/canbus/vehicle/teshun/teshun_controller.h"

#include "modules/common/proto/vehicle_signal.pb.h"

#include "modules/canbus/vehicle/teshun/teshun_message_manager.h"
#include "modules/canbus/vehicle/vehicle_controller.h"
#include "modules/common/log.h"
#include "modules/common/time/time.h"
#include "modules/drivers/canbus/can_comm/can_sender.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace jmc_auto {
namespace canbus {
namespace teshun {

using ::jmc_auto::drivers::canbus::ProtocolData;
using ::jmc_auto::common::ErrorCode;
using ::jmc_auto::control::ControlCommand;

namespace {

const int32_t kMaxFailAttempt = 10;
const int32_t CHECK_RESPONSE_STEER_UNIT_FLAG = 1;
const int32_t CHECK_RESPONSE_SPEED_UNIT_FLAG = 2;
const int32_t CHECK_RESPONSE_GEAR_UNIT_FLAG = 3;
}

ErrorCode TeshunController::Init(
	const VehicleParameter& params,
	CanSender<::jmc_auto::canbus::ChassisDetail> *const can_sender,
    MessageManager<::jmc_auto::canbus::ChassisDetail> *const message_manager) {
  if (is_initialized_) {
    AINFO << "TeshunController has already been initiated.";
    return ErrorCode::CANBUS_ERROR;
  }

  vehicle_params_.CopyFrom(
    common::VehicleConfigHelper::instance()->GetConfig().vehicle_param());
  params_.CopyFrom(params);
  if (!params_.has_driving_mode()) {
    AERROR << "Vehicle conf pb not set driving_mode.";
    return ErrorCode::CANBUS_ERROR;
  }

  if (can_sender == nullptr) {
    return ErrorCode::CANBUS_ERROR;
  }
  can_sender_ = can_sender;

  if (message_manager == nullptr) {
    AERROR << "protocol manager is null.";
    return ErrorCode::CANBUS_ERROR;
  }
  message_manager_ = message_manager;

  // sender part
  adu_bodycontrol_0x333_333_ = dynamic_cast<Adubodycontrol0x333333*>
          (message_manager_->GetMutableProtocolDataById(Adubodycontrol0x333333::ID));
  if (adu_bodycontrol_0x333_333_ == nullptr) {
     AERROR << "Adubodycontrol0x333333 does not exist in the TeshunMessageManager!";
     return ErrorCode::CANBUS_ERROR;
  }

  adu_controlbrake_0x110_110_ = dynamic_cast<Aducontrolbrake0x110110*>
          (message_manager_->GetMutableProtocolDataById(Aducontrolbrake0x110110::ID));
  if (adu_controlbrake_0x110_110_ == nullptr) {
     AERROR << "Aducontrolbrake0x110110 does not exist in the TeshunMessageManager!";
     return ErrorCode::CANBUS_ERROR;
  }

  adu_controldrive_0x120_120_ = dynamic_cast<Aducontroldrive0x120120*>
          (message_manager_->GetMutableProtocolDataById(Aducontroldrive0x120120::ID));
  if (adu_controldrive_0x120_120_ == nullptr) {
     AERROR << "Aducontroldrive0x120120 does not exist in the TeshunMessageManager!";
     return ErrorCode::CANBUS_ERROR;
  }

  adu_controleps2_0x100_100_ = dynamic_cast<Aducontroleps20x100100*>
          (message_manager_->GetMutableProtocolDataById(Aducontroleps20x100100::ID));
  if (adu_controleps2_0x100_100_ == nullptr) {
     AERROR << "Aducontroleps20x100100 does not exist in the TeshunMessageManager!";
     return ErrorCode::CANBUS_ERROR;
  }

  can_sender_->AddMessage(Adubodycontrol0x333333::ID, adu_bodycontrol_0x333_333_, false);
  can_sender_->AddMessage(Aducontrolbrake0x110110::ID, adu_controlbrake_0x110_110_, false);
  can_sender_->AddMessage(Aducontroldrive0x120120::ID, adu_controldrive_0x120_120_, false);
  can_sender_->AddMessage(Aducontroleps20x100100::ID, adu_controleps2_0x100_100_, false);

  // need sleep to ensure all messages received
  AINFO << "TeshunController is initialized.";

  is_initialized_ = true;
  return ErrorCode::OK;
}

TeshunController::~TeshunController() {}

bool TeshunController::Start() {
  if (!is_initialized_) {
    AERROR << "TeshunController has NOT been initiated.";
    return false;
  }
  const auto& update_func = [this] { SecurityDogThreadFunc(); };
  thread_.reset(new std::thread(update_func));

  return true;
}

void TeshunController::Stop() {
  if (!is_initialized_) {
    AERROR << "TeshunController stops or starts improperly!";
    return;
  }

  if (thread_ != nullptr && thread_->joinable()) {
    thread_->join();
    thread_.reset();
    AINFO << "TeshunController stopped.";
  }
}

Chassis TeshunController::chassis() {
  chassis_.Clear();

  ChassisDetail chassis_detail;
  message_manager_->GetSensorData(&chassis_detail);

  // 21, 22, previously 1, 2
  if (driving_mode() == Chassis::EMERGENCY_MODE) {
    set_chassis_error_code(Chassis::NO_ERROR);
  }

  chassis_.set_driving_mode(driving_mode());
  chassis_.set_error_code(chassis_error_code());

  // 3
  chassis_.set_engine_started(true);
  /* ADD YOUR OWN CAR CHASSIS OPERATION
  */
 /////////speed
 if (chassis_detail.teshun().has_gw_abs_sts_0x221_221() &&
      chassis_detail.teshun().gw_abs_sts_0x221_221().has_abs_vehspdlgt()) {
    chassis_.set_speed_mps(
        static_cast<float>(chassis_detail.teshun().gw_abs_sts_0x221_221().abs_vehspdlgt()));
  } else {
    chassis_.set_speed_mps(0);
  }

//////////throttle
if (chassis_detail.teshun().has_gw_mcu_output_0x225_225() &&
      chassis_detail.teshun().gw_mcu_output_0x225_225().has_mcu_trq_meas()) {
    chassis_.set_throttle_percentage(static_cast<float>(
        chassis_detail.teshun().gw_mcu_output_0x225_225().mcu_trq_meas()));
  } else {
    chassis_.set_throttle_percentage(0);
  }

//////////brake
if (chassis_detail.teshun().has_ibc_status_0x122_122() &&
      chassis_detail.teshun().ibc_status_0x122_122().has_ibc_brkpedalrawposition()) {
    chassis_.set_brake_percentage(static_cast<float>(
        chassis_detail.teshun().ibc_status_0x122_122().has_ibc_brkpedalrawposition()));
  } else {
    chassis_.set_brake_percentage(0);
  }
  
///////////gear
if (chassis_detail.teshun().has_gw_scu_shiftersts_0xc8_c8() &&
      chassis_detail.teshun().gw_scu_shiftersts_0xc8_c8().has_shifterposition()) {
    Chassis::GearPosition gear_pos = Chassis::GEAR_INVALID;

    if (chassis_detail.teshun().gw_scu_shiftersts_0xc8_c8().shifterposition() ==
        Gw_scu_shiftersts_0xc8_c8::SHIFTERPOSITION_NEUTRAL) {
      gear_pos = Chassis::GEAR_NEUTRAL;
    }
    if (chassis_detail.teshun().gw_scu_shiftersts_0xc8_c8().shifterposition() ==
        Gw_scu_shiftersts_0xc8_c8::SHIFTERPOSITION_REVERSE) {
      gear_pos = Chassis::GEAR_REVERSE;
    }
    if (chassis_detail.teshun().gw_scu_shiftersts_0xc8_c8().shifterposition() ==
        Gw_scu_shiftersts_0xc8_c8::SHIFTERPOSITION_DRIVE) {
      gear_pos = Chassis::GEAR_DRIVE;
    }
    if (chassis_detail.teshun().gw_scu_shiftersts_0xc8_c8().shifterposition() ==
        Gw_scu_shiftersts_0xc8_c8::SHIFTERPOSITION_PARK) {
      gear_pos = Chassis::GEAR_PARKING;
    }

    chassis_.set_gear_location(gear_pos);
  } else {
    chassis_.set_gear_location(Chassis::GEAR_NONE);
  }

/////////////steer
if (chassis_detail.teshun().has_eps2_status_0x112_112() &&
      chassis_detail.teshun().eps2_status_0x112_112().has_eps_steeringwheelang()) {
    chassis_.set_steering_percentage(static_cast<float>(
        chassis_detail.teshun().eps2_status_0x112_112().eps_steeringwheelang() * 100.0 /
        vehicle_params_.max_steer_angle()));
  } else {
    chassis_.set_steering_percentage(0);
  }

if (chassis_error_mask_) {
    chassis_.set_chassis_error_mask(chassis_error_mask_);
  }

  if (chassis_detail.has_surround()) {
    chassis_.mutable_surround()->CopyFrom(chassis_detail.surround());
  }
  // give engage_advice based on error_code and canbus feedback
  if (!chassis_error_mask_ && !chassis_.parking_brake() &&
      (chassis_.throttle_percentage() == 0.0)) {
    chassis_.mutable_engage_advice()->set_advice(
        jmc_auto::common::EngageAdvice::READY_TO_ENGAGE);
  } else {
    chassis_.mutable_engage_advice()->set_advice(
        jmc_auto::common::EngageAdvice::DISALLOW_ENGAGE);
    chassis_.mutable_engage_advice()->set_reason(
        "CANBUS not ready, firmware error or emergency button pressed!");
  }

  return chassis_;
}

void TeshunController::Emergency() {
  set_driving_mode(Chassis::EMERGENCY_MODE);
  ResetProtocol();
}

ErrorCode TeshunController::EnableAutoMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE) {
    AINFO << "already in COMPLETE_AUTO_DRIVE mode";
    return ErrorCode::OK;
  }
  adu_controleps2_0x100_100_->set_adu_controepsenable(
      Adu_controleps2_0x100_100::ADU_CONTROEPSENABLE_ENABLE);
  adu_controlbrake_0x110_110_->set_adu_controbrk_enable(
      Adu_controlbrake_0x110_110::ADU_CONTROBRK_ENABLE_ENABLE);
  adu_controldrive_0x120_120_->set_adu_controtorque_enable(
      Adu_controldrive_0x120_120::ADU_CONTROTORQUE_ENABLE_ENABLE);
  adu_controldrive_0x120_120_->set_adu_targetgear_enable(
      Adu_controldrive_0x120_120::ADU_TARGETGEAR_ENABLE_ENABLE);
  
  AINFO << "\n\n\n set enable \n\n\n";
  can_sender_->Update();
  const int32_t flag =
      CHECK_RESPONSE_STEER_UNIT_FLAG | CHECK_RESPONSE_SPEED_UNIT_FLAG  | CHECK_RESPONSE_GEAR_UNIT_FLAG;
  if (!CheckResponse(flag, true)) {
    AERROR << "Failed to switch to COMPLETE_AUTO_DRIVE mode.";
    Emergency();
    set_chassis_error_code(Chassis::CHASSIS_ERROR);
    return ErrorCode::CANBUS_ERROR;
  } else {
    set_driving_mode(Chassis::COMPLETE_AUTO_DRIVE);
    AINFO << "Switch to COMPLETE_AUTO_DRIVE mode ok.";
    return ErrorCode::OK;
  }
  
}

ErrorCode TeshunController::DisableAutoMode() {
	adu_controleps2_0x100_100_->set_adu_controepsenable(
		Adu_controleps2_0x100_100::ADU_CONTROEPSENABLE_DISABLE);
	adu_controlbrake_0x110_110_->set_adu_controbrk_enable(
		Adu_controlbrake_0x110_110::ADU_CONTROBRK_ENABLE_DISABLE);
	adu_controldrive_0x120_120_->set_adu_controtorque_enable(
		Adu_controldrive_0x120_120::ADU_CONTROTORQUE_ENABLE_DISABLE);
	adu_controldrive_0x120_120_->set_adu_targetgear_enable(
		Adu_controldrive_0x120_120::ADU_TARGETGEAR_ENABLE_DISABLE);
  ResetProtocol();
  can_sender_->Update();
  set_driving_mode(Chassis::COMPLETE_MANUAL);
  set_chassis_error_code(Chassis::NO_ERROR);
  AINFO << "Switch to COMPLETE_MANUAL ok.";
  return ErrorCode::OK;
}

ErrorCode TeshunController::EnableSteeringOnlyMode() {
  //if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
  //    driving_mode() == Chassis::AUTO_STEER_ONLY) {
  //  set_driving_mode(Chassis::AUTO_STEER_ONLY);
    AINFO << "Already in AUTO_STEER_ONLY mode";
    return ErrorCode::OK;
  //}
  //return ErrorCode::OK;
  /* ADD YOUR OWN CAR CHASSIS OPERATION
  brake_60_->set_disable();
  throttle_62_->set_disable();
  steering_64_->set_enable();

  can_sender_->Update();
  if (CheckResponse(CHECK_RESPONSE_STEER_UNIT_FLAG, true) == false) {
    AERROR << "Failed to switch to AUTO_STEER_ONLY mode.";
    Emergency();
    set_chassis_error_code(Chassis::CHASSIS_ERROR);
    return ErrorCode::CANBUS_ERROR;
  } else {
    set_driving_mode(Chassis::AUTO_STEER_ONLY);
    AINFO << "Switch to AUTO_STEER_ONLY mode ok.";
    return ErrorCode::OK;
  }
  */
}
ErrorCode TeshunController::EnableRemoteMode() {
if (driving_mode() == Chassis::REMOTE_MODE) {
    AINFO << "already in REMOTE_MODE mode";
    return ErrorCode::OK;
  }
 adu_controleps2_0x100_100_->set_adu_controepsenable(
		Adu_controleps2_0x100_100::ADU_CONTROEPSENABLE_DISABLE);
	adu_controlbrake_0x110_110_->set_adu_controbrk_enable(
		Adu_controlbrake_0x110_110::ADU_CONTROBRK_ENABLE_DISABLE);
	adu_controldrive_0x120_120_->set_adu_controtorque_enable(
		Adu_controldrive_0x120_120::ADU_CONTROTORQUE_ENABLE_DISABLE);
	adu_controldrive_0x120_120_->set_adu_targetgear_enable(
		Adu_controldrive_0x120_120::ADU_TARGETGEAR_ENABLE_DISABLE);
  ResetProtocol();
  can_sender_->Update();
  set_driving_mode(Chassis::COMPLETE_MANUAL);
  set_chassis_error_code(Chassis::NO_ERROR);
  AINFO << "Switch to COMPLETE_MANUAL ok.";
  return ErrorCode::OK;

}
//cx75在vechicle_control中添加的虚函数实现
ErrorCode TeshunController::EnableAPAMode() {}
ErrorCode TeshunController::DisableAPAMode() {}
void TeshunController::SteerTorque(double torque) {};
void TeshunController::PamStopDistance(int distance) {};
void TeshunController::SpeedTarget(float speed){};

ErrorCode TeshunController::EnableSpeedOnlyMode() {
 // if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
 //     driving_mode() == Chassis::AUTO_SPEED_ONLY) {
 //   set_driving_mode(Chassis::AUTO_SPEED_ONLY);
    AINFO << "Already in AUTO_SPEED_ONLY mode";
    return ErrorCode::OK;
 // }
 // return ErrorCode::OK;
  /* ADD YOUR OWN CAR CHASSIS OPERATION
  brake_60_->set_enable();
  throttle_62_->set_enable();
  steering_64_->set_disable();

  can_sender_->Update();
  if (CheckResponse(CHECK_RESPONSE_SPEED_UNIT_FLAG, true) == false) {
    AERROR << "Failed to switch to AUTO_STEER_ONLY mode.";
    Emergency();
    set_chassis_error_code(Chassis::CHASSIS_ERROR);
    return ErrorCode::CANBUS_ERROR;
  } else {
    set_driving_mode(Chassis::AUTO_SPEED_ONLY);
    AINFO << "Switch to AUTO_SPEED_ONLY mode ok.";
    return ErrorCode::OK;
  }
  */
}

// NEUTRAL, REVERSE, DRIVE
void TeshunController::Gear(Chassis::GearPosition gear_position) {
  if (!(driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
        driving_mode() == Chassis::AUTO_SPEED_ONLY)) {
    AINFO << "this drive mode no need to set gear.";
    return;
  }
 switch (gear_position) {
    case Chassis::GEAR_NEUTRAL: {
      adu_controldrive_0x120_120_->set_adu_targetgear_position(Adu_controldrive_0x120_120::ADU_TARGETGEAR_POSITION_N_NEUTRAL);
      break;
    }
    case Chassis::GEAR_REVERSE: {
      adu_controldrive_0x120_120_->set_adu_targetgear_position(Adu_controldrive_0x120_120::ADU_TARGETGEAR_POSITION_R_REVERSE);
      break;
    }
    case Chassis::GEAR_DRIVE: {
      adu_controldrive_0x120_120_->set_adu_targetgear_position(Adu_controldrive_0x120_120::ADU_TARGETGEAR_POSITION_D_DRIVE);
       break;
    }
    case Chassis::GEAR_PARKING: {
      adu_controldrive_0x120_120_->set_adu_targetgear_position(Adu_controldrive_0x120_120::ADU_TARGETGEAR_POSITION_P_PARK);
      break;
    }
    case Chassis::GEAR_INVALID: {
      AERROR << "Gear command is invalid!";
      adu_controldrive_0x120_120_->set_adu_targetgear_position(Adu_controldrive_0x120_120::ADU_TARGETGEAR_POSITION_N_NEUTRAL);
       break;
    }
    default: {
      adu_controldrive_0x120_120_->set_adu_targetgear_position(Adu_controldrive_0x120_120::ADU_TARGETGEAR_POSITION_N_NEUTRAL);
      break;
    }
  }
  
}

// brake with new acceleration
// acceleration:0.00~99.99, unit:
// acceleration:0.0 ~ 7.0, unit:m/s^2
// acceleration_spd:60 ~ 100, suggest: 90
// -> pedal
void TeshunController::Brake(double pedal) {
  // double real_value = params_.max_acc() * acceleration / 100;
  // TODO Update brake value based on mode
  if (!(driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
        driving_mode() == Chassis::AUTO_SPEED_ONLY)) {
    AINFO << "The current drive mode does not need to set acceleration.";
    return;
  }
  adu_controlbrake_0x110_110_->set_adu_brktmcposition_req(static_cast<int>(pedal));
  IC_Rolling_counter_0x110++;
  if(IC_Rolling_counter_0x110>15)
  {
    IC_Rolling_counter_0x110=0;
  }
  adu_controlbrake_0x110_110_->set_ic_rolling_counter_0x110(IC_Rolling_counter_0x110);
}

// drive with old acceleration
// gas:0.00~99.99 unit:
void TeshunController::Throttle(double pedal) {
  if (!(driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
        driving_mode() == Chassis::AUTO_SPEED_ONLY)) {
    AINFO << "The current drive mode does not need to set acceleration.";
    return;
  }
  adu_controldrive_0x120_120_->set_adu_targetdrving_torque(static_cast<int>(pedal)*50);
   /*Rolling_counter_0x120++;
  if(Rolling_counter_0x120>15)
  {
    Rolling_counter_0x120=0;
  }*/
  adu_controldrive_0x120_120_->set_rolling_counter_0x120(Rolling_counter_0x120);
}

void TeshunController::Acceleration(double acc) {}
// teshun default, -470 ~ 470, left:+, right:-
// need to be compatible with control module, so reverse
// steering with old angle speed
// angle:-99.99~0.00~99.99, unit:, left:-, right:+
void TeshunController::Steer(double angle) {
  if (!(driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
        driving_mode() == Chassis::AUTO_STEER_ONLY)) {
    AINFO << "The current driving mode does not need to set steer.";
    return;
  }
   const double real_angle = vehicle_params_.max_steer_angle() * angle / 100.0;
  // reverse sign
  adu_controleps2_0x100_100_->set_adu_controsteeringwheelangle(real_angle);
 
}

// steering with new angle speed
// angle:-99.99~0.00~99.99, unit:, left:-, right:+
// angle_spd:0.00~99.99, unit:deg/s
void TeshunController::Steer(double angle, double angle_spd) {
  if (!(driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
        driving_mode() == Chassis::AUTO_STEER_ONLY)) {
    AINFO << "The current driving mode does not need to set steer.";
    return;
  }
  // ADD YOUR OWN CAR CHASSIS OPERATION
  const double real_angle = vehicle_params_.max_steer_angle() * angle / 100.0;
  // const double min_steer_angle_rate=vehicle_params_.min_steer_angle_rate();
  // const double max_steer_angle_rate=vehicle_params_.max_steer_angle_rate();
  // const double real_angle_spd = ProtocolData::BoundedValue(
  //    min_steer_angle_rate, max_steer_angle_rate,
  //    max_steer_angle_rate * angle_spd / 100.0);
      //const double real_angle_spd =vehicle_params_.max_steer_angle_rate() * angle_spd / 100.0;
      adu_controleps2_0x100_100_->set_adu_controsteeringwheelangle(real_angle);
      adu_controleps2_0x100_100_->set_adu_ctrsteeringwheelanglespeed(angle_spd);
      AINFO << "angle_spd:"<<angle_spd;
      ADU_100h_MessageCounter++;
      //if (ADU_100h_MessageCounter>15)
      //{
      //  ADU_100h_MessageCounter=0;
      //}
      
      //adu_controleps2_0x100_100_->set_adu_100h_messagecounter(ADU_100h_MessageCounter);
}

void TeshunController::SetEpbBreak(const ControlCommand& command) {
  /*if (command.parking_brake()) {
    // None
  } else {
    // None
  }*/
}

void TeshunController::SetBeam(const ControlCommand& command) {
 /* if (command.signal().high_beam()) {
    // None
  } else if (command.signal().low_beam()) {
    // None
  } else {
    // None
  }*/
}

void TeshunController::SetHorn(const ControlCommand& command) {
 /* if (command.signal().horn()) {
    // None
  } else {
    // None
  }*/
}

void TeshunController::SetTurningSignal(const ControlCommand& command) {
  // Set Turn Signal
  /* ADD YOUR OWN CAR CHASSIS OPERATION
  auto signal = command.signal().turn_signal();
  if (signal == Signal::TURN_LEFT) {
    turnsignal_68_->set_turn_left();
  } else if (signal == Signal::TURN_RIGHT) {
    turnsignal_68_->set_turn_right();
  } else {
    turnsignal_68_->set_turn_none();
  }
  */
}

void TeshunController::ResetProtocol() {
  message_manager_->ResetSendMessages();
}

bool TeshunController::CheckChassisError() {
  /* ADD YOUR OWN CAR CHASSIS OPERATION
  */
 ChassisDetail chassis_detail;
  message_manager_->GetSensorData(&chassis_detail);
  if (!chassis_detail.has_ch()) {
    AERROR_EVERY(100) << "ChassisDetail has NO ch vehicle info."
                      << chassis_detail.DebugString();
    return false;
  }
  Teshun teshun = chassis_detail.teshun();
  // steer motor fault
  if (teshun.has_eps2_status_0x112_112()) {
    if (Eps2_status_0x112_112::EPS_SASFAILURESTS_SENSOR_INFORMATION_INVALID__AN_INTERNAL_SENSOR_FAULT_OCCURRED ==
        teshun.eps2_status_0x112_112().eps_sasfailurests()) {
      return true;
    } 
    }
    // drive error

   // brake error
  if (teshun.has_ibc_status_0x122_122()) {
    if (Ibc_status_0x122_122::IBC_SYSTEMSTATUS_SYSTEM_FAULT ==
        teshun.ibc_status_0x122_122().ibc_systemstatus()) {
      return true;
    }
  }
  return false;
}

void TeshunController::SecurityDogThreadFunc() {
  int32_t vertical_ctrl_fail = 0;
  int32_t horizontal_ctrl_fail = 0;

  if (can_sender_ == nullptr) {
    AERROR << "Fail to run SecurityDogThreadFunc() because can_sender_ is "
              "nullptr.";
    return;
  }
  while (!can_sender_->IsRunning()) {
    std::this_thread::yield();
  }

  std::chrono::duration<double, std::micro> default_period{50000};
  int64_t start = 0;
  int64_t end = 0;
  while (can_sender_->IsRunning()) {
    start = ::jmc_auto::common::time::AsInt64<::jmc_auto::common::time::micros>(
        ::jmc_auto::common::time::Clock::Now());
    const Chassis::DrivingMode mode = driving_mode();
    bool emergency_mode = false;

    // 1. horizontal control check
    if ((mode == Chassis::COMPLETE_AUTO_DRIVE ||
         mode == Chassis::AUTO_STEER_ONLY) &&
        CheckResponse(CHECK_RESPONSE_STEER_UNIT_FLAG, false) == false) {
      ++horizontal_ctrl_fail;
      if (horizontal_ctrl_fail >= kMaxFailAttempt) {
        emergency_mode = true;
        set_chassis_error_code(Chassis::MANUAL_INTERVENTION);
      }
    } else {
      horizontal_ctrl_fail = 0;
    }

    // 2. vertical control check
    if ((mode == Chassis::COMPLETE_AUTO_DRIVE ||
         mode == Chassis::AUTO_SPEED_ONLY) &&
        CheckResponse(CHECK_RESPONSE_SPEED_UNIT_FLAG, false) == false) {
      ++vertical_ctrl_fail;
      if (vertical_ctrl_fail >= kMaxFailAttempt) {
        emergency_mode = true;
        set_chassis_error_code(Chassis::MANUAL_INTERVENTION);
      }
    } else {
      vertical_ctrl_fail = 0;
    }
    if (CheckChassisError()) {
      set_chassis_error_code(Chassis::CHASSIS_ERROR);
      emergency_mode = true;
    }

    if (emergency_mode && mode != Chassis::EMERGENCY_MODE) {
      set_driving_mode(Chassis::EMERGENCY_MODE);
      message_manager_->ResetSendMessages();
    }
    end = ::jmc_auto::common::time::AsInt64<::jmc_auto::common::time::micros>(
        ::jmc_auto::common::time::Clock::Now());
    std::chrono::duration<double, std::micro> elapsed{end - start};
    if (elapsed < default_period) {
      std::this_thread::sleep_for(default_period - elapsed);
    } else {
      AERROR
          << "Too much time consumption in TeshunController looping process:"
          << elapsed.count();
    }
  }
}

bool TeshunController::CheckResponse(const int32_t flags, bool need_wait) {
  /* ADD YOUR OWN CAR CHASSIS OPERATION
  */
  int32_t retry_num = 20;
  ChassisDetail chassis_detail;
  bool is_eps_online = false;
  bool is_vcu_online = false;
  bool is_esp_online = false;
  bool is_switch_online = false;
  do {
    if (message_manager_->GetSensorData(&chassis_detail) != ErrorCode::OK) {
      AERROR_EVERY(100) << "get chassis detail failed.";
      return false;
    }
    bool check_ok = true;
    if (flags & CHECK_RESPONSE_STEER_UNIT_FLAG) {
      is_eps_online = chassis_detail.has_check_response() &&
                      chassis_detail.check_response().has_is_eps_online() &&
                      chassis_detail.check_response().is_eps_online();
      check_ok = check_ok && is_eps_online;
    }

    if (flags & CHECK_RESPONSE_SPEED_UNIT_FLAG) {
      is_vcu_online = chassis_detail.has_check_response() &&
                      chassis_detail.check_response().has_is_vcu_online() &&
                      chassis_detail.check_response().is_vcu_online();
      is_esp_online = chassis_detail.has_check_response() &&
                      chassis_detail.check_response().has_is_esp_online() &&
                      chassis_detail.check_response().is_esp_online();
      check_ok = check_ok && is_vcu_online && is_esp_online;
    }

    if (flags & CHECK_RESPONSE_GEAR_UNIT_FLAG) {
      is_switch_online = chassis_detail.has_check_response() &&
                      chassis_detail.check_response().has_is_switch_online() &&
                      chassis_detail.check_response().is_switch_online();
      check_ok = check_ok && is_switch_online;
    }

    if (check_ok) {
      return true;
    } else {
      AINFO << "Need to check response again.";
    }
    if (need_wait) {
      --retry_num;
      std::this_thread::sleep_for(
          std::chrono::duration<double, std::milli>(20));
    }
  } while (need_wait && retry_num);

  AINFO << "check_response fail: is_eps_online:" << is_eps_online
        << ", is_vcu_online:" << is_vcu_online
        << ", is_esp_online:" << is_esp_online
        << ", is_switch_online:" << is_switch_online;
  return false;
}


void TeshunController::set_chassis_error_mask(const int32_t mask) {
  std::lock_guard<std::mutex> lock(chassis_mask_mutex_);
  chassis_error_mask_ = mask;
}

int32_t TeshunController::chassis_error_mask() {
  std::lock_guard<std::mutex> lock(chassis_mask_mutex_);
  return chassis_error_mask_;
}

Chassis::ErrorCode TeshunController::chassis_error_code() {
  std::lock_guard<std::mutex> lock(chassis_error_code_mutex_);
  return chassis_error_code_;
}

void TeshunController::set_chassis_error_code(
    const Chassis::ErrorCode& error_code) {
  std::lock_guard<std::mutex> lock(chassis_error_code_mutex_);
  chassis_error_code_ = error_code;
}

}  // namespace teshun
}  // namespace canbus
}  // namespace jmc_auto
