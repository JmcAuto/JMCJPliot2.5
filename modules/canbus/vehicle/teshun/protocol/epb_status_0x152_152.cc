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

#include "modules/canbus/vehicle/teshun/protocol/epb_status_0x152_152.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace jmc_auto {
namespace canbus {
namespace teshun {

using ::jmc_auto::drivers::canbus::Byte;

Epbstatus0x152152::Epbstatus0x152152() {}
const int32_t Epbstatus0x152152::ID = 0x152;

void Epbstatus0x152152::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_teshun()->mutable_epb_status_0x152_152()->set_checksum_0x152(checksum_0x152(bytes, length));
  chassis->mutable_teshun()->mutable_epb_status_0x152_152()->set_rolling_counter_0x152(rolling_counter_0x152(bytes, length));
  chassis->mutable_teshun()->mutable_epb_status_0x152_152()->set_epb_faultcode(epb_faultcode(bytes, length));
  chassis->mutable_teshun()->mutable_epb_status_0x152_152()->set_epb_brakelampreq(epb_brakelampreq(bytes, length));
  chassis->mutable_teshun()->mutable_epb_status_0x152_152()->set_epb_sysfaultstatus(epb_sysfaultstatus(bytes, length));
  chassis->mutable_teshun()->mutable_epb_status_0x152_152()->set_epb_parkingstatus(epb_parkingstatus(bytes, length));
  chassis->mutable_teshun()->mutable_epb_status_0x152_152()->set_epb_switchstatus(epb_switchstatus(bytes, length));
}

// config detail: {'name': 'checksum_0x152', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Epbstatus0x152152::checksum_0x152(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'rolling_counter_0x152', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Epbstatus0x152152::rolling_counter_0x152(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'epb_faultcode', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 47, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Epbstatus0x152152::epb_faultcode(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'epb_brakelampreq', 'enum': {0: 'EPB_BRAKELAMPREQ_BRAKE_LAMP_OFF', 1: 'EPB_BRAKELAMPREQ_BRAKE_LAMP_ON'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 15, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Epb_status_0x152_152::Epb_brakelampreqType Epbstatus0x152152::epb_brakelampreq(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(7, 1);

  Epb_status_0x152_152::Epb_brakelampreqType ret =  static_cast<Epb_status_0x152_152::Epb_brakelampreqType>(x);
  return ret;
}

// config detail: {'name': 'epb_sysfaultstatus', 'enum': {0: 'EPB_SYSFAULTSTATUS_NO_FAULT', 1: 'EPB_SYSFAULTSTATUS_WARNING', 2: 'EPB_SYSFAULTSTATUS_FAULT', 3: 'EPB_SYSFAULTSTATUS_RESEVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|2]', 'bit': 2, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Epb_status_0x152_152::Epb_sysfaultstatusType Epbstatus0x152152::epb_sysfaultstatus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(1, 2);

  Epb_status_0x152_152::Epb_sysfaultstatusType ret =  static_cast<Epb_status_0x152_152::Epb_sysfaultstatusType>(x);
  return ret;
}

// config detail: {'name': 'epb_parkingstatus', 'enum': {0: 'EPB_PARKINGSTATUS_RELEASED', 1: 'EPB_PARKINGSTATUS_RELEASE_ONGOING', 2: 'EPB_PARKINGSTATUS_PARK_ONGOING', 3: 'EPB_PARKINGSTATUS_PARKED', 4: 'EPB_PARKINGSTATUS_UNKOWN'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|4]', 'bit': 5, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Epb_status_0x152_152::Epb_parkingstatusType Epbstatus0x152152::epb_parkingstatus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(3, 3);

  Epb_status_0x152_152::Epb_parkingstatusType ret =  static_cast<Epb_status_0x152_152::Epb_parkingstatusType>(x);
  return ret;
}

// config detail: {'name': 'epb_switchstatus', 'enum': {0: 'EPB_SWITCHSTATUS_NO_ACTION_ON_SWITCH', 1: 'EPB_SWITCHSTATUS_SWITCH_TO_RELEASE', 2: 'EPB_SWITCHSTATUS_SWITCH_TO_PARK', 3: 'EPB_SWITCHSTATUS_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|2]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Epb_status_0x152_152::Epb_switchstatusType Epbstatus0x152152::epb_switchstatus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(6, 2);

  Epb_status_0x152_152::Epb_switchstatusType ret =  static_cast<Epb_status_0x152_152::Epb_switchstatusType>(x);
  return ret;
}
}  // namespace teshun
}  // namespace canbus
}  // namespace jmc_auto
