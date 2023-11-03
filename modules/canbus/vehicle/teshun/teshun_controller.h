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

#ifndef MODULES_CANBUS_VEHICLE_TESHUN_TESHUN_PROTOCOL_MANAGER_H_
#define MODULES_CANBUS_VEHICLE_TESHUN_TESHUN_PROTOCOL_MANAGER_H_

#include <memory>
#include <thread>

#include "modules/canbus/vehicle/vehicle_controller.h"

#include "modules/canbus/proto/canbus_conf.pb.h"
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/canbus/proto/vehicle_parameter.pb.h"
#include "modules/common/macro.h"
#include "modules/common/proto/error_code.pb.h"
#include "modules/control/proto/control_cmd.pb.h"

#include "modules/canbus/vehicle/teshun/protocol/adu_bodycontrol_0x333_333.h"
#include "modules/canbus/vehicle/teshun/protocol/adu_controlbrake_0x110_110.h"
#include "modules/canbus/vehicle/teshun/protocol/adu_controldrive_0x120_120.h"
#include "modules/canbus/vehicle/teshun/protocol/adu_controleps2_0x100_100.h"

namespace jmc_auto {
namespace canbus {
namespace teshun {

class TeshunController final : public VehicleController {
 public:

  explicit TeshunController() {};

  virtual ~TeshunController();

  ::jmc_auto::common::ErrorCode Init(
      const VehicleParameter& params,
      CanSender<::jmc_auto::canbus::ChassisDetail> *const can_sender,
      MessageManager<::jmc_auto::canbus::ChassisDetail> *const message_manager) override;

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

 private:
  // main logical function for operation the car enter or exit the auto driving
  void Emergency() override;
  ::jmc_auto::common::ErrorCode EnableAutoMode() override;
  ::jmc_auto::common::ErrorCode DisableAutoMode() override;
  ::jmc_auto::common::ErrorCode EnableSteeringOnlyMode() override;
  ::jmc_auto::common::ErrorCode EnableSpeedOnlyMode() override;
  ::jmc_auto::common::ErrorCode EnableRemoteMode();

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
  Adubodycontrol0x333333* adu_bodycontrol_0x333_333_ = nullptr;
  Aducontrolbrake0x110110* adu_controlbrake_0x110_110_ = nullptr;
  Aducontroldrive0x120120* adu_controldrive_0x120_120_ = nullptr;
  Aducontroleps20x100100* adu_controleps2_0x100_100_ = nullptr;

  Chassis chassis_;
  std::unique_ptr<std::thread> thread_;
  bool is_chassis_error_ = false;
int ADU_100h_MessageCounter  =0;
  int IC_Rolling_counter_0x110 =0;
  int Rolling_counter_0x120    =0;

  std::mutex chassis_error_code_mutex_;
  Chassis::ErrorCode chassis_error_code_ = Chassis::NO_ERROR;

  std::mutex chassis_mask_mutex_;
  int32_t chassis_error_mask_ = 0;
};

}  // namespace teshun
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICLE_TESHUN_TESHUN_PROTOCOL_MANAGER_H_
