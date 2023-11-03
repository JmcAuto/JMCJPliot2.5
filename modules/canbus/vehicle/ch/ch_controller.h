/******************************************************************************
 * Copyright 2019 The JmcAuto Authors. All Rights Reserved.
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

#pragma once

#include <memory>
#include <thread>

#include "modules/common/macro.h"
#include "modules/canbus/proto/canbus_conf.pb.h"
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/canbus/proto/vehicle_parameter.pb.h"
#include "modules/canbus/vehicle/vehicle_controller.h"
#include "modules/common/proto/error_code.pb.h"
#include "modules/control/proto/control_cmd.pb.h"

#include "modules/canbus/vehicle/ch/protocol/brake_command_111.h"
#include "modules/canbus/vehicle/ch/protocol/control_command_115.h"
#include "modules/canbus/vehicle/ch/protocol/gear_command_114.h"
#include "modules/canbus/vehicle/ch/protocol/steer_command_112.h"
#include "modules/canbus/vehicle/ch/protocol/throttle_command_110.h"
#include "modules/canbus/vehicle/ch/protocol/turnsignal_command_113.h"

namespace jmc_auto {
namespace canbus {
namespace ch {

class ChController final : public VehicleController {
 public:
  ChController() {}

  virtual ~ChController();

  ::jmc_auto::common::ErrorCode Init(
      const VehicleParameter& params,
      CanSender<::jmc_auto::canbus::ChassisDetail>* const can_sender,
      MessageManager<::jmc_auto::canbus::ChassisDetail>* const message_manager)
      override;

  bool Start() override;

  /**
   * @brief stop the vehicle controller.
   */
  void Stop() override;

  /**
   * @brief calculate and return the chassis.
   * @returns a copy of chassis. Use copy here to avoid multi-thread issues.
   */
  Chassis chassis() override;
  FRIEND_TEST(ChControllerTest, SetDrivingMode);
  FRIEND_TEST(ChControllerTest, Status);
  FRIEND_TEST(ChControllerTest, UpdateDrivingMode);

 private:
  // main logical function for operation the car enter or exit the auto driving
  void Emergency() override;
  ::jmc_auto::common::ErrorCode EnableAutoMode() override;
  ::jmc_auto::common::ErrorCode DisableAutoMode() override;
  ::jmc_auto::common::ErrorCode EnableSteeringOnlyMode() override;
  ::jmc_auto::common::ErrorCode EnableSpeedOnlyMode() override;
  ::jmc_auto::common::ErrorCode EnableRemoteMode() override;


//cx75在vechicle_control中添加的虚函数实现
  ::jmc_auto::common::ErrorCode EnableAPAMode() override;
  ::jmc_auto::common::ErrorCode DisableAPAMode() override;
    void SteerTorque(double torque) override;
    void PamStopDistance(int distance) override;
    void SpeedTarget(float speed) override;
  // NEUTRAL, REVERSE, DRIVE
  void Gear(Chassis::GearPosition state) override;

  // brake with new acceleration
  // acceleration:0.00~99.99, unit:
  // acceleration_spd: 60 ~ 100, suggest: 90
  void Brake(double acceleration) override;

  // drive with old acceleration
  // gas:0.00~99.99 unit:
  void Throttle(double throttle) override;

  void Acceleration(double acc) override;

  // steering with old angle speed
  // angle:-99.99~0.00~99.99, unit:, left:+, right:-
  void Steer(double angle) override;

  // steering with new angle speed
  // angle:-99.99~0.00~99.99, unit:, left:+, right:-
  // angle_spd:0.00~99.99, unit:deg/s
  void Steer(double angle, double angle_spd) override;

  // set Electrical Park Brake
  void SetEpbBreak(const ::jmc_auto::control::ControlCommand& command) override;
  void SetBeam(const ::jmc_auto::control::ControlCommand& command) override;
  void SetHorn(const ::jmc_auto::control::ControlCommand& command) override;
  void SetTurningSignal(
      const ::jmc_auto::control::ControlCommand& command) override;

  void ResetProtocol();
  bool CheckChassisError();

 private:
  void SecurityDogThreadFunc();
  virtual bool CheckResponse(const int32_t flags, bool need_wait);
  void set_chassis_error_mask(const int32_t mask);
  int32_t chassis_error_mask();
  Chassis::ErrorCode chassis_error_code();
  void set_chassis_error_code(const Chassis::ErrorCode& error_code);

 private:
  // control protocol
  Brakecommand111* brake_command_111_ = nullptr;
  Controlcommand115* control_command_115_ = nullptr;
  Gearcommand114* gear_command_114_ = nullptr;
  Steercommand112* steer_command_112_ = nullptr;
  Throttlecommand110* throttle_command_110_ = nullptr;
  Turnsignalcommand113* turnsignal_command_113_ = nullptr;

  Chassis chassis_;
  std::unique_ptr<std::thread> thread_;
  bool is_chassis_error_ = false;

  std::mutex chassis_error_code_mutex_;
  Chassis::ErrorCode chassis_error_code_ = Chassis::NO_ERROR;

  std::mutex chassis_mask_mutex_;
  int32_t chassis_error_mask_ = 0;
};

}  // namespace ch
}  // namespace canbus
}  // namespace jmc_auto
