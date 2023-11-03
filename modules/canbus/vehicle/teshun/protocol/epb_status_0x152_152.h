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

#ifndef MODULES_CANBUS_VEHICLE_TESHUN_PROTOCOL_EPB_STATUS_0X152_152_H_
#define MODULES_CANBUS_VEHICLE_TESHUN_PROTOCOL_EPB_STATUS_0X152_152_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace jmc_auto {
namespace canbus {
namespace teshun {

class Epbstatus0x152152 : public ::jmc_auto::drivers::canbus::ProtocolData<
                    ::jmc_auto::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Epbstatus0x152152();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'name': 'Checksum_0x152', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int checksum_0x152(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Rolling_counter_0x152', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int rolling_counter_0x152(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'EPB_FaultCode', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 47, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int epb_faultcode(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'EPB_BrakeLampReq', 'enum': {0: 'EPB_BRAKELAMPREQ_BRAKE_LAMP_OFF', 1: 'EPB_BRAKELAMPREQ_BRAKE_LAMP_ON'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 15, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Epb_status_0x152_152::Epb_brakelampreqType epb_brakelampreq(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'EPB_SysFaultStatus', 'enum': {0: 'EPB_SYSFAULTSTATUS_NO_FAULT', 1: 'EPB_SYSFAULTSTATUS_WARNING', 2: 'EPB_SYSFAULTSTATUS_FAULT', 3: 'EPB_SYSFAULTSTATUS_RESEVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|2]', 'bit': 2, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Epb_status_0x152_152::Epb_sysfaultstatusType epb_sysfaultstatus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'EPB_ParkingStatus', 'enum': {0: 'EPB_PARKINGSTATUS_RELEASED', 1: 'EPB_PARKINGSTATUS_RELEASE_ONGOING', 2: 'EPB_PARKINGSTATUS_PARK_ONGOING', 3: 'EPB_PARKINGSTATUS_PARKED', 4: 'EPB_PARKINGSTATUS_UNKOWN'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|4]', 'bit': 5, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Epb_status_0x152_152::Epb_parkingstatusType epb_parkingstatus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'EPB_SwitchStatus', 'enum': {0: 'EPB_SWITCHSTATUS_NO_ACTION_ON_SWITCH', 1: 'EPB_SWITCHSTATUS_SWITCH_TO_RELEASE', 2: 'EPB_SWITCHSTATUS_SWITCH_TO_PARK', 3: 'EPB_SWITCHSTATUS_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|2]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Epb_status_0x152_152::Epb_switchstatusType epb_switchstatus(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace teshun
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_TESHUN_PROTOCOL_EPB_STATUS_0X152_152_H_
