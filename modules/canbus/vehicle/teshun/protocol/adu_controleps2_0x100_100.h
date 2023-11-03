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

#ifndef MODULES_CANBUS_VEHICLE_TESHUN_PROTOCOL_ADU_CONTROLEPS2_0X100_100_H_
#define MODULES_CANBUS_VEHICLE_TESHUN_PROTOCOL_ADU_CONTROLEPS2_0X100_100_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace jmc_auto {
namespace canbus {
namespace teshun {

class Aducontroleps20x100100 : public ::jmc_auto::drivers::canbus::ProtocolData<
                    ::jmc_auto::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Aducontroleps20x100100();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'name': 'ADU_100h_MessageChecksum', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  Aducontroleps20x100100* set_adu_100h_messagechecksum(int adu_100h_messagechecksum);

  // config detail: {'name': 'ADU_100h_MessageCounter', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  Aducontroleps20x100100* set_adu_100h_messagecounter(int adu_100h_messagecounter);

  // config detail: {'name': 'ADU_CtrSteeringwheelanglespeed', 'offset': 0.0, 'precision': 1.0, 'len': 9, 'is_signed_var': False, 'physical_range': '[0|510]', 'bit': 31, 'type': 'int', 'order': 'motorola', 'physical_unit': 'deg/s'}
  Aducontroleps20x100100* set_adu_ctrsteeringwheelanglespeed(int adu_ctrsteeringwheelanglespeed);

  // config detail: {'name': 'ADU_ControSteeringwheelangle', 'offset': -612.0, 'precision': 0.1, 'len': 16, 'is_signed_var': False, 'physical_range': '[-612|612]', 'bit': 15, 'type': 'double', 'order': 'motorola', 'physical_unit': 'degree'}
  Aducontroleps20x100100* set_adu_controsteeringwheelangle(double adu_controsteeringwheelangle);

  // config detail: {'name': 'ADU_ControEpsEnable', 'enum': {0: 'ADU_CONTROEPSENABLE_DISABLE', 1: 'ADU_CONTROEPSENABLE_ENABLE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 0, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Aducontroleps20x100100* set_adu_controepsenable(Adu_controleps2_0x100_100::Adu_controepsenableType adu_controepsenable);

 private:

  // config detail: {'name': 'ADU_100h_MessageChecksum', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  void set_p_adu_100h_messagechecksum(uint8_t* data, int adu_100h_messagechecksum);

  // config detail: {'name': 'ADU_100h_MessageCounter', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  void set_p_adu_100h_messagecounter(uint8_t* data, int adu_100h_messagecounter);

  // config detail: {'name': 'ADU_CtrSteeringwheelanglespeed', 'offset': 0.0, 'precision': 1.0, 'len': 9, 'is_signed_var': False, 'physical_range': '[0|510]', 'bit': 31, 'type': 'int', 'order': 'motorola', 'physical_unit': 'deg/s'}
  void set_p_adu_ctrsteeringwheelanglespeed(uint8_t* data, int adu_ctrsteeringwheelanglespeed);

  // config detail: {'name': 'ADU_ControSteeringwheelangle', 'offset': -612.0, 'precision': 0.1, 'len': 16, 'is_signed_var': False, 'physical_range': '[-612|612]', 'bit': 15, 'type': 'double', 'order': 'motorola', 'physical_unit': 'degree'}
  void set_p_adu_controsteeringwheelangle(uint8_t* data, double adu_controsteeringwheelangle);

  // config detail: {'name': 'ADU_ControEpsEnable', 'enum': {0: 'ADU_CONTROEPSENABLE_DISABLE', 1: 'ADU_CONTROEPSENABLE_ENABLE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 0, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_adu_controepsenable(uint8_t* data, Adu_controleps2_0x100_100::Adu_controepsenableType adu_controepsenable);

 private:
  int adu_100h_messagechecksum_;
  int adu_100h_messagecounter_;
  int adu_ctrsteeringwheelanglespeed_;
  double adu_controsteeringwheelangle_;
  Adu_controleps2_0x100_100::Adu_controepsenableType adu_controepsenable_;
};

}  // namespace teshun
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_TESHUN_PROTOCOL_ADU_CONTROLEPS2_0X100_100_H_
