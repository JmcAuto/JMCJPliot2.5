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

#include "modules/canbus/vehicle/teshun/protocol/ibc_status_0x122_122.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace jmc_auto {
namespace canbus {
namespace teshun {

using ::jmc_auto::drivers::canbus::Byte;

Ibcstatus0x122122::Ibcstatus0x122122() {}
const int32_t Ibcstatus0x122122::ID = 0x122;

void Ibcstatus0x122122::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_teshun()->mutable_ibc_status_0x122_122()->set_checksum_0x122(checksum_0x122(bytes, length));
  chassis->mutable_teshun()->mutable_ibc_status_0x122_122()->set_rolling_counter_0x122(rolling_counter_0x122(bytes, length));
  chassis->mutable_teshun()->mutable_ibc_status_0x122_122()->set_ibc_parkrelease_req(ibc_parkrelease_req(bytes, length));
  chassis->mutable_teshun()->mutable_ibc_status_0x122_122()->set_ibc_faultcode(ibc_faultcode(bytes, length));
  chassis->mutable_teshun()->mutable_ibc_status_0x122_122()->set_ibc_brkpedalrawposition(ibc_brkpedalrawposition(bytes, length));
  chassis->mutable_teshun()->mutable_ibc_status_0x122_122()->set_ibc_mastercylinderpressvalid(ibc_mastercylinderpressvalid(bytes, length));
  chassis->mutable_teshun()->mutable_ibc_status_0x122_122()->set_ibc_mastercylinderpress(ibc_mastercylinderpress(bytes, length));
  chassis->mutable_teshun()->mutable_ibc_status_0x122_122()->set_ibc_brakeactive(ibc_brakeactive(bytes, length));
  chassis->mutable_teshun()->mutable_ibc_status_0x122_122()->set_ibc_brakereqavailabe(ibc_brakereqavailabe(bytes, length));
  chassis->mutable_teshun()->mutable_ibc_status_0x122_122()->set_ibc_brakepressurereqack(ibc_brakepressurereqack(bytes, length));
  chassis->mutable_teshun()->mutable_ibc_status_0x122_122()->set_ibc_controlstatus(ibc_controlstatus(bytes, length));
  chassis->mutable_teshun()->mutable_ibc_status_0x122_122()->set_ibc_driver_intervene(ibc_driver_intervene(bytes, length));
  chassis->mutable_teshun()->mutable_ibc_status_0x122_122()->set_ibc_systemstatus(ibc_systemstatus(bytes, length));

chassis->mutable_check_response()->set_is_esp_online(
      ibc_controlstatus(bytes, length) == 2);
}

// config detail: {'name': 'checksum_0x122', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Ibcstatus0x122122::checksum_0x122(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'rolling_counter_0x122', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Ibcstatus0x122122::rolling_counter_0x122(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'ibc_parkrelease_req', 'enum': {0: 'IBC_PARKRELEASE_REQ_NO_CONTROL', 1: 'IBC_PARKRELEASE_REQ_RELEASE', 2: 'IBC_PARKRELEASE_REQ_PARK', 3: 'IBC_PARKRELEASE_REQ_DYNAMIC_PARKING'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 55, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Ibc_status_0x122_122::Ibc_parkrelease_reqType Ibcstatus0x122122::ibc_parkrelease_req(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(6, 2);

  Ibc_status_0x122_122::Ibc_parkrelease_reqType ret =  static_cast<Ibc_status_0x122_122::Ibc_parkrelease_reqType>(x);
  return ret;
}

// config detail: {'name': 'ibc_faultcode', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 47, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Ibcstatus0x122122::ibc_faultcode(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'ibc_brkpedalrawposition', 'offset': 0.0, 'precision': 0.3937, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|99.9998]', 'bit': 31, 'type': 'double', 'order': 'motorola', 'physical_unit': '%'}
double Ibcstatus0x122122::ibc_brkpedalrawposition(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 0.393700;
  return ret;
}

// config detail: {'name': 'ibc_mastercylinderpressvalid', 'enum': {0: 'IBC_MASTERCYLINDERPRESSVALID_INVAILD', 1: 'IBC_MASTERCYLINDERPRESSVALID_VALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 16, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Ibc_status_0x122_122::Ibc_mastercylinderpressvalidType Ibcstatus0x122122::ibc_mastercylinderpressvalid(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 1);

  Ibc_status_0x122_122::Ibc_mastercylinderpressvalidType ret =  static_cast<Ibc_status_0x122_122::Ibc_mastercylinderpressvalidType>(x);
  return ret;
}

// config detail: {'name': 'ibc_mastercylinderpress', 'offset': 0.0, 'precision': 1.0, 'len': 15, 'is_signed_var': False, 'physical_range': '[0|32000]', 'bit': 15, 'type': 'int', 'order': 'motorola', 'physical_unit': 'kpa'}
int Ibcstatus0x122122::ibc_mastercylinderpress(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(1, 7);
  x <<= 7;
  x |= t;

  int ret = x;
  return ret;
}

// config detail: {'name': 'ibc_brakeactive', 'enum': {0: 'IBC_BRAKEACTIVE_BRAKE_INACTIVE', 1: 'IBC_BRAKEACTIVE_BRAKE_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 0, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Ibc_status_0x122_122::Ibc_brakeactiveType Ibcstatus0x122122::ibc_brakeactive(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 1);

  Ibc_status_0x122_122::Ibc_brakeactiveType ret =  static_cast<Ibc_status_0x122_122::Ibc_brakeactiveType>(x);
  return ret;
}

// config detail: {'name': 'ibc_brakereqavailabe', 'enum': {0: 'IBC_BRAKEREQAVAILABE_BRAKE_NOT_AVAILABLE', 1: 'IBC_BRAKEREQAVAILABE_BRAKE_AVAILABLE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 1, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Ibc_status_0x122_122::Ibc_brakereqavailabeType Ibcstatus0x122122::ibc_brakereqavailabe(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(1, 1);

  Ibc_status_0x122_122::Ibc_brakereqavailabeType ret =  static_cast<Ibc_status_0x122_122::Ibc_brakereqavailabeType>(x);
  return ret;
}

// config detail: {'name': 'ibc_brakepressurereqack', 'enum': {0: 'IBC_BRAKEPRESSUREREQACK_NOT_ACK', 1: 'IBC_BRAKEPRESSUREREQACK_ACK'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 2, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Ibc_status_0x122_122::Ibc_brakepressurereqackType Ibcstatus0x122122::ibc_brakepressurereqack(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(2, 1);

  Ibc_status_0x122_122::Ibc_brakepressurereqackType ret =  static_cast<Ibc_status_0x122_122::Ibc_brakepressurereqackType>(x);
  return ret;
}

// config detail: {'name': 'ibc_controlstatus', 'enum': {0: 'IBC_CONTROLSTATUS_TEMPORARILY_INHIBIT', 1: 'IBC_CONTROLSTATUS_AVAILABLE_FOR_CONTROL', 2: 'IBC_CONTROLSTATUS_ACTIVE', 3: 'IBC_CONTROLSTATUS_PERMANENTLY_FAILED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|2]', 'bit': 4, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Ibc_status_0x122_122::Ibc_controlstatusType Ibcstatus0x122122::ibc_controlstatus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(3, 2);

  Ibc_status_0x122_122::Ibc_controlstatusType ret =  static_cast<Ibc_status_0x122_122::Ibc_controlstatusType>(x);
  return ret;
}

// config detail: {'name': 'ibc_driver_intervene', 'enum': {0: 'IBC_DRIVER_INTERVENE_NOT_INTERVENE', 1: 'IBC_DRIVER_INTERVENE_INTERVENE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 5, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Ibc_status_0x122_122::Ibc_driver_interveneType Ibcstatus0x122122::ibc_driver_intervene(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(5, 1);

  Ibc_status_0x122_122::Ibc_driver_interveneType ret =  static_cast<Ibc_status_0x122_122::Ibc_driver_interveneType>(x);
  return ret;
}

// config detail: {'name': 'ibc_systemstatus', 'enum': {0: 'IBC_SYSTEMSTATUS_SYSTEM_NO_FAULT', 1: 'IBC_SYSTEMSTATUS_SYSTEM_WARING', 2: 'IBC_SYSTEMSTATUS_SYSTEM_FAULT', 3: 'IBC_SYSTEMSTATUS_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Ibc_status_0x122_122::Ibc_systemstatusType Ibcstatus0x122122::ibc_systemstatus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(6, 2);

  Ibc_status_0x122_122::Ibc_systemstatusType ret =  static_cast<Ibc_status_0x122_122::Ibc_systemstatusType>(x);
  return ret;
}
}  // namespace teshun
}  // namespace canbus
}  // namespace jmc_auto
