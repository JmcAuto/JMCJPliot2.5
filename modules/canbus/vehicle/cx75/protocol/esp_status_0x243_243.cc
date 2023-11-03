/******************************************************************************
 * Copyright 2017 The jmc_auto Authors. All Rights Reserved.
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

#include "modules/canbus/vehicle/cx75/protocol/esp_status_0x243_243.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

using ::jmc_auto::drivers::canbus::Byte;

Espstatus0x243243::Espstatus0x243243() {}
const int32_t Espstatus0x243243::ID = 0x243;

void Espstatus0x243243::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_cx75()->mutable_esp_status_0x243_243()->set_esp_hhcavailable(esp_hhcavailable(bytes, length));
  chassis->mutable_cx75()->mutable_esp_status_0x243_243()->set_esp_hhcactive(esp_hhcactive(bytes, length));
  chassis->mutable_cx75()->mutable_esp_status_0x243_243()->set_esp_hdcactive(esp_hdcactive(bytes, length));
  chassis->mutable_cx75()->mutable_esp_status_0x243_243()->set_esp_ldmblc_blrequestcontroller(esp_ldmblc_blrequestcontroller(bytes, length));
  chassis->mutable_cx75()->mutable_esp_status_0x243_243()->set_esp_cdpavailable(esp_cdpavailable(bytes, length));
  chassis->mutable_cx75()->mutable_esp_status_0x243_243()->set_esp_cdpactive(esp_cdpactive(bytes, length));
  chassis->mutable_cx75()->mutable_esp_status_0x243_243()->set_esp_epbwarningmessage3(esp_epbwarningmessage3(bytes, length));
  chassis->mutable_cx75()->mutable_esp_status_0x243_243()->set_esp_epbwarningmessage2(esp_epbwarningmessage2(bytes, length));
  chassis->mutable_cx75()->mutable_esp_status_0x243_243()->set_esp_epbwarningmessage1(esp_epbwarningmessage1(bytes, length));
  chassis->mutable_cx75()->mutable_esp_status_0x243_243()->set_esp_avhwarningmessage(esp_avhwarningmessage(bytes, length));
  chassis->mutable_cx75()->mutable_esp_status_0x243_243()->set_esp_avhactive(esp_avhactive(bytes, length));
  chassis->mutable_cx75()->mutable_esp_status_0x243_243()->set_esp_avhstandby(esp_avhstandby(bytes, length));
  chassis->mutable_cx75()->mutable_esp_status_0x243_243()->set_esp_avhavabliable(esp_avhavabliable(bytes, length));
  chassis->mutable_cx75()->mutable_esp_status_0x243_243()->set_esp_vdcflgflt(esp_vdcflgflt(bytes, length));
  chassis->mutable_cx75()->mutable_esp_status_0x243_243()->set_esp_tcsflgflt(esp_tcsflgflt(bytes, length));
  chassis->mutable_cx75()->mutable_esp_status_0x243_243()->set_esp_tcsctrlactv(esp_tcsctrlactv(bytes, length));
  chassis->mutable_cx75()->mutable_esp_status_0x243_243()->set_esp_espctrlactv(esp_espctrlactv(bytes, length));
  chassis->mutable_cx75()->mutable_esp_status_0x243_243()->set_esp_driving_mode_req(esp_driving_mode_req(bytes, length));
  chassis->mutable_cx75()->mutable_esp_status_0x243_243()->set_esp_espinfolamp(esp_espinfolamp(bytes, length));
  chassis->mutable_cx75()->mutable_esp_status_0x243_243()->set_esp_espenablests(esp_espenablests(bytes, length));
  chassis->mutable_cx75()->mutable_esp_status_0x243_243()->set_esp_gearshiftprevn(esp_gearshiftprevn(bytes, length));
  chassis->mutable_cx75()->mutable_esp_status_0x243_243()->set_esp_epbstatus(esp_epbstatus(bytes, length));
  chassis->mutable_cx75()->mutable_esp_status_0x243_243()->set_rollingcounter_0x243(rollingcounter_0x243(bytes, length));
  chassis->mutable_cx75()->mutable_esp_status_0x243_243()->set_rollerbenchmodests(rollerbenchmodests(bytes, length));
  chassis->mutable_cx75()->mutable_esp_status_0x243_243()->set_checksum_0x243(checksum_0x243(bytes, length));
  chassis->mutable_cx75()->mutable_esp_status_0x243_243()->set_esp_epberrorstatus(esp_epberrorstatus(bytes, length));
  chassis->mutable_cx75()->mutable_esp_status_0x243_243()->set_esp_ccswitchoff(esp_ccswitchoff(bytes, length));
  chassis->mutable_cx75()->mutable_esp_status_0x243_243()->set_esp_hdcavailable(esp_hdcavailable(bytes, length));
}

// config detail: {'name': 'esp_hhcavailable', 'enum': {0: 'ESP_HHCAVAILABLE_AVAILABLE', 1: 'ESP_HHCAVAILABLE_NOT_AVAILABLE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 0, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_status_0x243_243::Esp_hhcavailableType Espstatus0x243243::esp_hhcavailable(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 1);

  Esp_status_0x243_243::Esp_hhcavailableType ret =  static_cast<Esp_status_0x243_243::Esp_hhcavailableType>(x);
  return ret;
}

// config detail: {'name': 'esp_hhcactive', 'enum': {0: 'ESP_HHCACTIVE_NOT_ACTIVE', 1: 'ESP_HHCACTIVE_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 1, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_status_0x243_243::Esp_hhcactiveType Espstatus0x243243::esp_hhcactive(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(1, 1);

  Esp_status_0x243_243::Esp_hhcactiveType ret =  static_cast<Esp_status_0x243_243::Esp_hhcactiveType>(x);
  return ret;
}

// config detail: {'name': 'esp_hdcactive', 'enum': {0: 'ESP_HDCACTIVE_OFF', 1: 'ESP_HDCACTIVE_ON_ACTIVE_BRAKING', 2: 'ESP_HDCACTIVE_ON_NOT_ACTIVE_BRAKING'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|2]', 'bit': 12, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_status_0x243_243::Esp_hdcactiveType Espstatus0x243243::esp_hdcactive(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(3, 2);

  Esp_status_0x243_243::Esp_hdcactiveType ret =  static_cast<Esp_status_0x243_243::Esp_hdcactiveType>(x);
  return ret;
}

// config detail: {'name': 'esp_ldmblc_blrequestcontroller', 'enum': {0: 'ESP_LDMBLC_BLREQUESTCONTROLLER_NOT_ILLUMINATE', 1: 'ESP_LDMBLC_BLREQUESTCONTROLLER_ILLUMINATE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 13, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_status_0x243_243::Esp_ldmblc_blrequestcontrollerType Espstatus0x243243::esp_ldmblc_blrequestcontroller(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(5, 1);

  Esp_status_0x243_243::Esp_ldmblc_blrequestcontrollerType ret =  static_cast<Esp_status_0x243_243::Esp_ldmblc_blrequestcontrollerType>(x);
  return ret;
}

// config detail: {'name': 'esp_cdpavailable', 'enum': {0: 'ESP_CDPAVAILABLE_AVAILABLE', 1: 'ESP_CDPAVAILABLE_NOT_AVAILABLE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 14, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_status_0x243_243::Esp_cdpavailableType Espstatus0x243243::esp_cdpavailable(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(6, 1);

  Esp_status_0x243_243::Esp_cdpavailableType ret =  static_cast<Esp_status_0x243_243::Esp_cdpavailableType>(x);
  return ret;
}

// config detail: {'name': 'esp_cdpactive', 'enum': {0: 'ESP_CDPACTIVE_NOT_ACTIVE', 1: 'ESP_CDPACTIVE_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 15, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_status_0x243_243::Esp_cdpactiveType Espstatus0x243243::esp_cdpactive(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(7, 1);

  Esp_status_0x243_243::Esp_cdpactiveType ret =  static_cast<Esp_status_0x243_243::Esp_cdpactiveType>(x);
  return ret;
}

// config detail: {'name': 'esp_epbwarningmessage3', 'enum': {0: 'ESP_EPBWARNINGMESSAGE3_NO_DISPLAY', 1: 'ESP_EPBWARNINGMESSAGE3_WHEN_APPLIED_THE_EPB_THE_SLOPE_OF_VEHICLE_IS_MORE_THAN_20_FOR_PK_30_FOR_SUV'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 17, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_status_0x243_243::Esp_epbwarningmessage3Type Espstatus0x243243::esp_epbwarningmessage3(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(1, 1);

  Esp_status_0x243_243::Esp_epbwarningmessage3Type ret =  static_cast<Esp_status_0x243_243::Esp_epbwarningmessage3Type>(x);
  return ret;
}

// config detail: {'name': 'esp_epbwarningmessage2', 'enum': {0: 'ESP_EPBWARNINGMESSAGE2_NO_DISPLAY', 1: 'ESP_EPBWARNINGMESSAGE2_EPB_DRIVEROFF_WITHOUT_SEATBLET_OR_DRIVER_DOOR'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 18, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_status_0x243_243::Esp_epbwarningmessage2Type Espstatus0x243243::esp_epbwarningmessage2(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(2, 1);

  Esp_status_0x243_243::Esp_epbwarningmessage2Type ret =  static_cast<Esp_status_0x243_243::Esp_epbwarningmessage2Type>(x);
  return ret;
}

// config detail: {'name': 'esp_epbwarningmessage1', 'enum': {0: 'ESP_EPBWARNINGMESSAGE1_NO_DISPLAY', 1: 'ESP_EPBWARNINGMESSAGE1_NO_BRAKE_PEDAL_PRESSED_AND_RELEASE_THE_EPB'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 19, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_status_0x243_243::Esp_epbwarningmessage1Type Espstatus0x243243::esp_epbwarningmessage1(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(3, 1);

  Esp_status_0x243_243::Esp_epbwarningmessage1Type ret =  static_cast<Esp_status_0x243_243::Esp_epbwarningmessage1Type>(x);
  return ret;
}

// config detail: {'name': 'esp_avhwarningmessage', 'enum': {0: 'ESP_AVHWARNINGMESSAGE_NO_WARNING', 1: 'ESP_AVHWARNINGMESSAGE_PLEASE_CLOSE_THE_DOOR_OR_SEATBLET_THEN_PRESSURE_THE_AVH_BUTTON'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 20, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_status_0x243_243::Esp_avhwarningmessageType Espstatus0x243243::esp_avhwarningmessage(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(4, 1);

  Esp_status_0x243_243::Esp_avhwarningmessageType ret =  static_cast<Esp_status_0x243_243::Esp_avhwarningmessageType>(x);
  return ret;
}

// config detail: {'name': 'esp_avhactive', 'enum': {0: 'ESP_AVHACTIVE_INACTIVE', 1: 'ESP_AVHACTIVE_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 21, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_status_0x243_243::Esp_avhactiveType Espstatus0x243243::esp_avhactive(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(5, 1);

  Esp_status_0x243_243::Esp_avhactiveType ret =  static_cast<Esp_status_0x243_243::Esp_avhactiveType>(x);
  return ret;
}

// config detail: {'name': 'esp_avhstandby', 'enum': {0: 'ESP_AVHSTANDBY_NOT_STANDBY', 1: 'ESP_AVHSTANDBY_STANDBY'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 22, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_status_0x243_243::Esp_avhstandbyType Espstatus0x243243::esp_avhstandby(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(6, 1);

  Esp_status_0x243_243::Esp_avhstandbyType ret =  static_cast<Esp_status_0x243_243::Esp_avhstandbyType>(x);
  return ret;
}

// config detail: {'name': 'esp_avhavabliable', 'enum': {0: 'ESP_AVHAVABLIABLE_AVAILABLE', 1: 'ESP_AVHAVABLIABLE_NOT_AVAILABLE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 23, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_status_0x243_243::Esp_avhavabliableType Espstatus0x243243::esp_avhavabliable(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(7, 1);

  Esp_status_0x243_243::Esp_avhavabliableType ret =  static_cast<Esp_status_0x243_243::Esp_avhavabliableType>(x);
  return ret;
}

// config detail: {'name': 'esp_vdcflgflt', 'enum': {0: 'ESP_VDCFLGFLT_NO_FAILURE', 1: 'ESP_VDCFLGFLT_FAILURE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 32, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_status_0x243_243::Esp_vdcflgfltType Espstatus0x243243::esp_vdcflgflt(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 1);

  Esp_status_0x243_243::Esp_vdcflgfltType ret =  static_cast<Esp_status_0x243_243::Esp_vdcflgfltType>(x);
  return ret;
}

// config detail: {'name': 'esp_tcsflgflt', 'enum': {0: 'ESP_TCSFLGFLT_NO_FAILURE', 1: 'ESP_TCSFLGFLT_FAILURE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 33, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_status_0x243_243::Esp_tcsflgfltType Espstatus0x243243::esp_tcsflgflt(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(1, 1);

  Esp_status_0x243_243::Esp_tcsflgfltType ret =  static_cast<Esp_status_0x243_243::Esp_tcsflgfltType>(x);
  return ret;
}

// config detail: {'name': 'esp_tcsctrlactv', 'enum': {0: 'ESP_TCSCTRLACTV_NOT_ACTIVE', 1: 'ESP_TCSCTRLACTV_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 34, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_status_0x243_243::Esp_tcsctrlactvType Espstatus0x243243::esp_tcsctrlactv(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(2, 1);

  Esp_status_0x243_243::Esp_tcsctrlactvType ret =  static_cast<Esp_status_0x243_243::Esp_tcsctrlactvType>(x);
  return ret;
}

// config detail: {'name': 'esp_espctrlactv', 'enum': {0: 'ESP_ESPCTRLACTV_NOT_ACTIVE', 1: 'ESP_ESPCTRLACTV_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 35, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_status_0x243_243::Esp_espctrlactvType Espstatus0x243243::esp_espctrlactv(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(3, 1);

  Esp_status_0x243_243::Esp_espctrlactvType ret =  static_cast<Esp_status_0x243_243::Esp_espctrlactvType>(x);
  return ret;
}

// config detail: {'name': 'esp_driving_mode_req', 'enum': {0: 'ESP_DRIVING_MODE_REQ_AUTO', 1: 'ESP_DRIVING_MODE_REQ_SNOW', 2: 'ESP_DRIVING_MODE_REQ_NO_USED', 3: 'ESP_DRIVING_MODE_REQ_SPORT', 4: 'ESP_DRIVING_MODE_REQ_ECO', 5: 'ESP_DRIVING_MODE_REQ_4L', 6: 'ESP_DRIVING_MODE_REQ_ROCK', 7: 'ESP_DRIVING_MODE_REQ_WETMUD', 8: 'ESP_DRIVING_MODE_REQ_SAND', 13: 'ESP_DRIVING_MODE_REQ_ATS_SHIFTING', 14: 'ESP_DRIVING_MODE_REQ_ATS_SHIFT_FAILURE', 15: 'ESP_DRIVING_MODE_REQ_INVALID'}, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|15]', 'bit': 39, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_status_0x243_243::Esp_driving_mode_reqType Espstatus0x243243::esp_driving_mode_req(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(4, 4);

  Esp_status_0x243_243::Esp_driving_mode_reqType ret =  static_cast<Esp_status_0x243_243::Esp_driving_mode_reqType>(x);
  return ret;
}

// config detail: {'name': 'esp_espinfolamp', 'enum': {0: 'ESP_ESPINFOLAMP_OFF', 1: 'ESP_ESPINFOLAMP_ON'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 44, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_status_0x243_243::Esp_espinfolampType Espstatus0x243243::esp_espinfolamp(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(4, 1);

  Esp_status_0x243_243::Esp_espinfolampType ret =  static_cast<Esp_status_0x243_243::Esp_espinfolampType>(x);
  return ret;
}

// config detail: {'name': 'esp_espenablests', 'enum': {0: 'ESP_ESPENABLESTS_ESP_DISABLED', 1: 'ESP_ESPENABLESTS_ESP_ENABLED'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 45, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_status_0x243_243::Esp_espenablestsType Espstatus0x243243::esp_espenablests(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(5, 1);

  Esp_status_0x243_243::Esp_espenablestsType ret =  static_cast<Esp_status_0x243_243::Esp_espenablestsType>(x);
  return ret;
}

// config detail: {'name': 'esp_gearshiftprevn', 'enum': {0: 'ESP_GEARSHIFTPREVN_NO_REQUEST', 1: 'ESP_GEARSHIFTPREVN_INHIBIT_UPSHIFT', 2: 'ESP_GEARSHIFTPREVN_INHIBIT_DOWN_SHIFT', 3: 'ESP_GEARSHIFTPREVN_INHIBIT_ALL_SHIFTS'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 47, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_status_0x243_243::Esp_gearshiftprevnType Espstatus0x243243::esp_gearshiftprevn(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(6, 2);

  Esp_status_0x243_243::Esp_gearshiftprevnType ret =  static_cast<Esp_status_0x243_243::Esp_gearshiftprevnType>(x);
  return ret;
}

// config detail: {'name': 'esp_epbstatus', 'enum': {0: 'ESP_EPBSTATUS_RELEASED', 1: 'ESP_EPBSTATUS_APPLIED', 2: 'ESP_EPBSTATUS_RELEASINGR', 3: 'ESP_EPBSTATUS_APPLYING', 4: 'ESP_EPBSTATUS_UNKNOWN', 5: 'ESP_EPBSTATUS_ERROR'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 5, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_status_0x243_243::Esp_epbstatusType Espstatus0x243243::esp_epbstatus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(3, 3);

  Esp_status_0x243_243::Esp_epbstatusType ret =  static_cast<Esp_status_0x243_243::Esp_epbstatusType>(x);
  return ret;
}

// config detail: {'name': 'rollingcounter_0x243', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Espstatus0x243243::rollingcounter_0x243(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'rollerbenchmodests', 'enum': {0: 'ROLLERBENCHMODESTS_ROLLERBENCH_MODE_INACTIVE', 1: 'ROLLERBENCHMODESTS_ROLLERBENCH_MODE_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 52, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_status_0x243_243::RollerbenchmodestsType Espstatus0x243243::rollerbenchmodests(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(4, 1);

  Esp_status_0x243_243::RollerbenchmodestsType ret =  static_cast<Esp_status_0x243_243::RollerbenchmodestsType>(x);
  return ret;
}

// config detail: {'name': 'checksum_0x243', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Espstatus0x243243::checksum_0x243(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'esp_epberrorstatus', 'enum': {0: 'ESP_EPBERRORSTATUS_UNDEFINED', 1: 'ESP_EPBERRORSTATUS_NO_ERROR', 2: 'ESP_EPBERRORSTATUS_ERROR', 3: 'ESP_EPBERRORSTATUS_DIAGNOSIS'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_status_0x243_243::Esp_epberrorstatusType Espstatus0x243243::esp_epberrorstatus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(6, 2);

  Esp_status_0x243_243::Esp_epberrorstatusType ret =  static_cast<Esp_status_0x243_243::Esp_epberrorstatusType>(x);
  return ret;
}

// config detail: {'name': 'esp_ccswitchoff', 'enum': {0: 'ESP_CCSWITCHOFF_NOT_SWITCH_OFF_CRUISE_CONTROL', 1: 'ESP_CCSWITCHOFF_SWITCH_OFF_CRUISE_CONTROL'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 8, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_status_0x243_243::Esp_ccswitchoffType Espstatus0x243243::esp_ccswitchoff(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 1);

  Esp_status_0x243_243::Esp_ccswitchoffType ret =  static_cast<Esp_status_0x243_243::Esp_ccswitchoffType>(x);
  return ret;
}

// config detail: {'name': 'esp_hdcavailable', 'enum': {0: 'ESP_HDCAVAILABLE_AVAILABLE', 1: 'ESP_HDCAVAILABLE_NOT_AVAILABLE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 9, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_status_0x243_243::Esp_hdcavailableType Espstatus0x243243::esp_hdcavailable(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(1, 1);

  Esp_status_0x243_243::Esp_hdcavailableType ret =  static_cast<Esp_status_0x243_243::Esp_hdcavailableType>(x);
  return ret;
}
}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto
