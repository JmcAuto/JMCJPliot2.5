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

#ifndef MODULES_CANBUS_VEHICLE_TESHUN_PROTOCOL_ADU_CONTROLBRAKE_0X110_110_H_
#define MODULES_CANBUS_VEHICLE_TESHUN_PROTOCOL_ADU_CONTROLBRAKE_0X110_110_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace jmc_auto {
namespace canbus {
namespace teshun {

class Aducontrolbrake0x110110 : public ::jmc_auto::drivers::canbus::ProtocolData<
                    ::jmc_auto::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Aducontrolbrake0x110110();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'name': 'IC_Checksum_0x110', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  Aducontrolbrake0x110110* set_ic_checksum_0x110(int ic_checksum_0x110);

  // config detail: {'name': 'IC_Rolling_counter_0x110', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  Aducontrolbrake0x110110* set_ic_rolling_counter_0x110(int ic_rolling_counter_0x110);

  // config detail: {'name': 'ADU_Tgt_Deceleration', 'offset': 0.0, 'precision': 0.00159276, 'len': 12, 'is_signed_var': True, 'physical_range': '[0|0]', 'bit': 47, 'type': 'double', 'order': 'motorola', 'physical_unit': 'g'}
  Aducontrolbrake0x110110* set_adu_tgt_deceleration(double adu_tgt_deceleration);

  // config detail: {'name': 'ADU_BrkTMCPosition_Req', 'offset': 0.0, 'precision': 0.1, 'len': 10, 'is_signed_var': False, 'physical_range': '[0|100]', 'bit': 31, 'type': 'double', 'order': 'motorola', 'physical_unit': '%'}
  Aducontrolbrake0x110110* set_adu_brktmcposition_req(double adu_brktmcposition_req);

  // config detail: {'name': 'ADU_ParkRelease_Req', 'enum': {0: 'ADU_PARKRELEASE_REQ_NO_CONTROL', 1: 'ADU_PARKRELEASE_REQ_RELEASE', 2: 'ADU_PARKRELEASE_REQ_PARK', 3: 'ADU_PARKRELEASE_REQ_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 23, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Aducontrolbrake0x110110* set_adu_parkrelease_req(Adu_controlbrake_0x110_110::Adu_parkrelease_reqType adu_parkrelease_req);

  // config detail: {'name': 'ADU_ControBrk_StandStill', 'enum': {0: 'ADU_CONTROBRK_STANDSTILL_NOT_STANDSTILL', 1: 'ADU_CONTROBRK_STANDSTILL_STANDSTILL'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 20, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Aducontrolbrake0x110110* set_adu_controbrk_standstill(Adu_controlbrake_0x110_110::Adu_controbrk_standstillType adu_controbrk_standstill);

  // config detail: {'name': 'ADU_ControBrk_Enable', 'enum': {0: 'ADU_CONTROBRK_ENABLE_DISABLE', 1: 'ADU_CONTROBRK_ENABLE_ENABLE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 21, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Aducontrolbrake0x110110* set_adu_controbrk_enable(Adu_controlbrake_0x110_110::Adu_controbrk_enableType adu_controbrk_enable);

  // config detail: {'name': 'ADU_MasterCylinderPressReq', 'offset': 0.0, 'precision': 1.0, 'len': 15, 'is_signed_var': False, 'physical_range': '[0|32000]', 'bit': 7, 'type': 'int', 'order': 'motorola', 'physical_unit': 'kpa'}
  Aducontrolbrake0x110110* set_adu_mastercylinderpressreq(int adu_mastercylinderpressreq);

 private:

  // config detail: {'name': 'IC_Checksum_0x110', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  void set_p_ic_checksum_0x110(uint8_t* data, int ic_checksum_0x110);

  // config detail: {'name': 'IC_Rolling_counter_0x110', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  void set_p_ic_rolling_counter_0x110(uint8_t* data, int ic_rolling_counter_0x110);

  // config detail: {'name': 'ADU_Tgt_Deceleration', 'offset': 0.0, 'precision': 0.00159276, 'len': 12, 'is_signed_var': True, 'physical_range': '[0|0]', 'bit': 47, 'type': 'double', 'order': 'motorola', 'physical_unit': 'g'}
  void set_p_adu_tgt_deceleration(uint8_t* data, double adu_tgt_deceleration);

  // config detail: {'name': 'ADU_BrkTMCPosition_Req', 'offset': 0.0, 'precision': 0.1, 'len': 10, 'is_signed_var': False, 'physical_range': '[0|100]', 'bit': 31, 'type': 'double', 'order': 'motorola', 'physical_unit': '%'}
  void set_p_adu_brktmcposition_req(uint8_t* data, double adu_brktmcposition_req);

  // config detail: {'name': 'ADU_ParkRelease_Req', 'enum': {0: 'ADU_PARKRELEASE_REQ_NO_CONTROL', 1: 'ADU_PARKRELEASE_REQ_RELEASE', 2: 'ADU_PARKRELEASE_REQ_PARK', 3: 'ADU_PARKRELEASE_REQ_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 23, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_adu_parkrelease_req(uint8_t* data, Adu_controlbrake_0x110_110::Adu_parkrelease_reqType adu_parkrelease_req);

  // config detail: {'name': 'ADU_ControBrk_StandStill', 'enum': {0: 'ADU_CONTROBRK_STANDSTILL_NOT_STANDSTILL', 1: 'ADU_CONTROBRK_STANDSTILL_STANDSTILL'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 20, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_adu_controbrk_standstill(uint8_t* data, Adu_controlbrake_0x110_110::Adu_controbrk_standstillType adu_controbrk_standstill);

  // config detail: {'name': 'ADU_ControBrk_Enable', 'enum': {0: 'ADU_CONTROBRK_ENABLE_DISABLE', 1: 'ADU_CONTROBRK_ENABLE_ENABLE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 21, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_adu_controbrk_enable(uint8_t* data, Adu_controlbrake_0x110_110::Adu_controbrk_enableType adu_controbrk_enable);

  // config detail: {'name': 'ADU_MasterCylinderPressReq', 'offset': 0.0, 'precision': 1.0, 'len': 15, 'is_signed_var': False, 'physical_range': '[0|32000]', 'bit': 7, 'type': 'int', 'order': 'motorola', 'physical_unit': 'kpa'}
  void set_p_adu_mastercylinderpressreq(uint8_t* data, int adu_mastercylinderpressreq);

 private:
  int ic_checksum_0x110_;
  int ic_rolling_counter_0x110_;
  double adu_tgt_deceleration_;
  double adu_brktmcposition_req_;
  Adu_controlbrake_0x110_110::Adu_parkrelease_reqType adu_parkrelease_req_;
  Adu_controlbrake_0x110_110::Adu_controbrk_standstillType adu_controbrk_standstill_;
  Adu_controlbrake_0x110_110::Adu_controbrk_enableType adu_controbrk_enable_;
  int adu_mastercylinderpressreq_;
};

}  // namespace teshun
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_TESHUN_PROTOCOL_ADU_CONTROLBRAKE_0X110_110_H_
