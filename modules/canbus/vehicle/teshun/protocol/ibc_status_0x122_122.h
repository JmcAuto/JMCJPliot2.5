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

#ifndef MODULES_CANBUS_VEHICLE_TESHUN_PROTOCOL_IBC_STATUS_0X122_122_H_
#define MODULES_CANBUS_VEHICLE_TESHUN_PROTOCOL_IBC_STATUS_0X122_122_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace jmc_auto {
namespace canbus {
namespace teshun {

class Ibcstatus0x122122 : public ::jmc_auto::drivers::canbus::ProtocolData<
                    ::jmc_auto::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Ibcstatus0x122122();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'name': 'Checksum_0x122', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int checksum_0x122(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Rolling_counter_0x122', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int rolling_counter_0x122(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'IBC_ParkRelease_Req', 'enum': {0: 'IBC_PARKRELEASE_REQ_NO_CONTROL', 1: 'IBC_PARKRELEASE_REQ_RELEASE', 2: 'IBC_PARKRELEASE_REQ_PARK', 3: 'IBC_PARKRELEASE_REQ_DYNAMIC_PARKING'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 55, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Ibc_status_0x122_122::Ibc_parkrelease_reqType ibc_parkrelease_req(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'IBC_FaultCode', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 47, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int ibc_faultcode(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'IBC_BrkPedalRawPosition', 'offset': 0.0, 'precision': 0.3937, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|99.9998]', 'bit': 31, 'type': 'double', 'order': 'motorola', 'physical_unit': '%'}
  double ibc_brkpedalrawposition(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'IBC_MasterCylinderPressValid', 'enum': {0: 'IBC_MASTERCYLINDERPRESSVALID_INVAILD', 1: 'IBC_MASTERCYLINDERPRESSVALID_VALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 16, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Ibc_status_0x122_122::Ibc_mastercylinderpressvalidType ibc_mastercylinderpressvalid(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'IBC_MasterCylinderPress', 'offset': 0.0, 'precision': 1.0, 'len': 15, 'is_signed_var': False, 'physical_range': '[0|32000]', 'bit': 15, 'type': 'int', 'order': 'motorola', 'physical_unit': 'kpa'}
  int ibc_mastercylinderpress(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'IBC_BrakeActive', 'enum': {0: 'IBC_BRAKEACTIVE_BRAKE_INACTIVE', 1: 'IBC_BRAKEACTIVE_BRAKE_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 0, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Ibc_status_0x122_122::Ibc_brakeactiveType ibc_brakeactive(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'IBC_BrakeReqAvailabe', 'enum': {0: 'IBC_BRAKEREQAVAILABE_BRAKE_NOT_AVAILABLE', 1: 'IBC_BRAKEREQAVAILABE_BRAKE_AVAILABLE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 1, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Ibc_status_0x122_122::Ibc_brakereqavailabeType ibc_brakereqavailabe(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'IBC_BrakePressureReqACK', 'enum': {0: 'IBC_BRAKEPRESSUREREQACK_NOT_ACK', 1: 'IBC_BRAKEPRESSUREREQACK_ACK'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 2, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Ibc_status_0x122_122::Ibc_brakepressurereqackType ibc_brakepressurereqack(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'IBC_ControlStatus', 'enum': {0: 'IBC_CONTROLSTATUS_TEMPORARILY_INHIBIT', 1: 'IBC_CONTROLSTATUS_AVAILABLE_FOR_CONTROL', 2: 'IBC_CONTROLSTATUS_ACTIVE', 3: 'IBC_CONTROLSTATUS_PERMANENTLY_FAILED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|2]', 'bit': 4, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Ibc_status_0x122_122::Ibc_controlstatusType ibc_controlstatus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'IBC_Driver_Intervene', 'enum': {0: 'IBC_DRIVER_INTERVENE_NOT_INTERVENE', 1: 'IBC_DRIVER_INTERVENE_INTERVENE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 5, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Ibc_status_0x122_122::Ibc_driver_interveneType ibc_driver_intervene(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'IBC_SystemStatus', 'enum': {0: 'IBC_SYSTEMSTATUS_SYSTEM_NO_FAULT', 1: 'IBC_SYSTEMSTATUS_SYSTEM_WARING', 2: 'IBC_SYSTEMSTATUS_SYSTEM_FAULT', 3: 'IBC_SYSTEMSTATUS_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Ibc_status_0x122_122::Ibc_systemstatusType ibc_systemstatus(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace teshun
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_TESHUN_PROTOCOL_IBC_STATUS_0X122_122_H_
