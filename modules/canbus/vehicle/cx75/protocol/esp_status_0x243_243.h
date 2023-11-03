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

#ifndef MODULES_CANBUS_VEHICLE_cx75_PROTOCOL_ESP_STATUS_0X243_243_H_
#define MODULES_CANBUS_VEHICLE_cx75_PROTOCOL_ESP_STATUS_0X243_243_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

class Espstatus0x243243 : public ::jmc_auto::drivers::canbus::ProtocolData<
                    ::jmc_auto::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Espstatus0x243243();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'name': 'ESP_HHCAvailable', 'enum': {0: 'ESP_HHCAVAILABLE_AVAILABLE', 1: 'ESP_HHCAVAILABLE_NOT_AVAILABLE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 0, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_status_0x243_243::Esp_hhcavailableType esp_hhcavailable(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_HHCActive', 'enum': {0: 'ESP_HHCACTIVE_NOT_ACTIVE', 1: 'ESP_HHCACTIVE_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 1, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_status_0x243_243::Esp_hhcactiveType esp_hhcactive(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_HDCActive', 'enum': {0: 'ESP_HDCACTIVE_OFF', 1: 'ESP_HDCACTIVE_ON_ACTIVE_BRAKING', 2: 'ESP_HDCACTIVE_ON_NOT_ACTIVE_BRAKING'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|2]', 'bit': 12, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_status_0x243_243::Esp_hdcactiveType esp_hdcactive(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_LdmBLC_BLRequestController', 'enum': {0: 'ESP_LDMBLC_BLREQUESTCONTROLLER_NOT_ILLUMINATE', 1: 'ESP_LDMBLC_BLREQUESTCONTROLLER_ILLUMINATE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 13, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_status_0x243_243::Esp_ldmblc_blrequestcontrollerType esp_ldmblc_blrequestcontroller(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_CdpAvailable', 'enum': {0: 'ESP_CDPAVAILABLE_AVAILABLE', 1: 'ESP_CDPAVAILABLE_NOT_AVAILABLE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 14, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_status_0x243_243::Esp_cdpavailableType esp_cdpavailable(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_CdpActive', 'enum': {0: 'ESP_CDPACTIVE_NOT_ACTIVE', 1: 'ESP_CDPACTIVE_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 15, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_status_0x243_243::Esp_cdpactiveType esp_cdpactive(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_EPBWarningMessage3', 'enum': {0: 'ESP_EPBWARNINGMESSAGE3_NO_DISPLAY', 1: 'ESP_EPBWARNINGMESSAGE3_WHEN_APPLIED_THE_EPB_THE_SLOPE_OF_VEHICLE_IS_MORE_THAN_20_FOR_PK_30_FOR_SUV'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 17, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_status_0x243_243::Esp_epbwarningmessage3Type esp_epbwarningmessage3(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_EPBWarningMessage2', 'enum': {0: 'ESP_EPBWARNINGMESSAGE2_NO_DISPLAY', 1: 'ESP_EPBWARNINGMESSAGE2_EPB_DRIVEROFF_WITHOUT_SEATBLET_OR_DRIVER_DOOR'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 18, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_status_0x243_243::Esp_epbwarningmessage2Type esp_epbwarningmessage2(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_EPBWarningMessage1', 'enum': {0: 'ESP_EPBWARNINGMESSAGE1_NO_DISPLAY', 1: 'ESP_EPBWARNINGMESSAGE1_NO_BRAKE_PEDAL_PRESSED_AND_RELEASE_THE_EPB'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 19, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_status_0x243_243::Esp_epbwarningmessage1Type esp_epbwarningmessage1(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_AVHWarningMessage', 'enum': {0: 'ESP_AVHWARNINGMESSAGE_NO_WARNING', 1: 'ESP_AVHWARNINGMESSAGE_PLEASE_CLOSE_THE_DOOR_OR_SEATBLET_THEN_PRESSURE_THE_AVH_BUTTON'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 20, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_status_0x243_243::Esp_avhwarningmessageType esp_avhwarningmessage(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_AvhActive', 'enum': {0: 'ESP_AVHACTIVE_INACTIVE', 1: 'ESP_AVHACTIVE_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 21, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_status_0x243_243::Esp_avhactiveType esp_avhactive(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_AvhStandby', 'enum': {0: 'ESP_AVHSTANDBY_NOT_STANDBY', 1: 'ESP_AVHSTANDBY_STANDBY'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 22, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_status_0x243_243::Esp_avhstandbyType esp_avhstandby(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_AvhAvabliable', 'enum': {0: 'ESP_AVHAVABLIABLE_AVAILABLE', 1: 'ESP_AVHAVABLIABLE_NOT_AVAILABLE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 23, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_status_0x243_243::Esp_avhavabliableType esp_avhavabliable(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_VdcFlgFlt', 'enum': {0: 'ESP_VDCFLGFLT_NO_FAILURE', 1: 'ESP_VDCFLGFLT_FAILURE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 32, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_status_0x243_243::Esp_vdcflgfltType esp_vdcflgflt(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_TcsFlgFlt', 'enum': {0: 'ESP_TCSFLGFLT_NO_FAILURE', 1: 'ESP_TCSFLGFLT_FAILURE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 33, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_status_0x243_243::Esp_tcsflgfltType esp_tcsflgflt(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_TcsCtrlActv', 'enum': {0: 'ESP_TCSCTRLACTV_NOT_ACTIVE', 1: 'ESP_TCSCTRLACTV_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 34, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_status_0x243_243::Esp_tcsctrlactvType esp_tcsctrlactv(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_EspCtrlActv', 'enum': {0: 'ESP_ESPCTRLACTV_NOT_ACTIVE', 1: 'ESP_ESPCTRLACTV_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 35, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_status_0x243_243::Esp_espctrlactvType esp_espctrlactv(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_Driving_mode_req', 'enum': {0: 'ESP_DRIVING_MODE_REQ_AUTO', 1: 'ESP_DRIVING_MODE_REQ_SNOW', 2: 'ESP_DRIVING_MODE_REQ_NO_USED', 3: 'ESP_DRIVING_MODE_REQ_SPORT', 4: 'ESP_DRIVING_MODE_REQ_ECO', 5: 'ESP_DRIVING_MODE_REQ_4L', 6: 'ESP_DRIVING_MODE_REQ_ROCK', 7: 'ESP_DRIVING_MODE_REQ_WETMUD', 8: 'ESP_DRIVING_MODE_REQ_SAND', 13: 'ESP_DRIVING_MODE_REQ_ATS_SHIFTING', 14: 'ESP_DRIVING_MODE_REQ_ATS_SHIFT_FAILURE', 15: 'ESP_DRIVING_MODE_REQ_INVALID'}, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|15]', 'bit': 39, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_status_0x243_243::Esp_driving_mode_reqType esp_driving_mode_req(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_ESPInfoLamp', 'enum': {0: 'ESP_ESPINFOLAMP_OFF', 1: 'ESP_ESPINFOLAMP_ON'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 44, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_status_0x243_243::Esp_espinfolampType esp_espinfolamp(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_EspEnableSts', 'enum': {0: 'ESP_ESPENABLESTS_ESP_DISABLED', 1: 'ESP_ESPENABLESTS_ESP_ENABLED'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 45, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_status_0x243_243::Esp_espenablestsType esp_espenablests(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_GearShiftPrevn', 'enum': {0: 'ESP_GEARSHIFTPREVN_NO_REQUEST', 1: 'ESP_GEARSHIFTPREVN_INHIBIT_UPSHIFT', 2: 'ESP_GEARSHIFTPREVN_INHIBIT_DOWN_SHIFT', 3: 'ESP_GEARSHIFTPREVN_INHIBIT_ALL_SHIFTS'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 47, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_status_0x243_243::Esp_gearshiftprevnType esp_gearshiftprevn(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_EPBStatus', 'enum': {0: 'ESP_EPBSTATUS_RELEASED', 1: 'ESP_EPBSTATUS_APPLIED', 2: 'ESP_EPBSTATUS_RELEASINGR', 3: 'ESP_EPBSTATUS_APPLYING', 4: 'ESP_EPBSTATUS_UNKNOWN', 5: 'ESP_EPBSTATUS_ERROR'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 5, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_status_0x243_243::Esp_epbstatusType esp_epbstatus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'RollingCounter_0x243', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int rollingcounter_0x243(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'RollerBenchModeSts', 'enum': {0: 'ROLLERBENCHMODESTS_ROLLERBENCH_MODE_INACTIVE', 1: 'ROLLERBENCHMODESTS_ROLLERBENCH_MODE_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 52, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_status_0x243_243::RollerbenchmodestsType rollerbenchmodests(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Checksum_0x243', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int checksum_0x243(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_EPBErrorStatus', 'enum': {0: 'ESP_EPBERRORSTATUS_UNDEFINED', 1: 'ESP_EPBERRORSTATUS_NO_ERROR', 2: 'ESP_EPBERRORSTATUS_ERROR', 3: 'ESP_EPBERRORSTATUS_DIAGNOSIS'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_status_0x243_243::Esp_epberrorstatusType esp_epberrorstatus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_CCSwitchOff', 'enum': {0: 'ESP_CCSWITCHOFF_NOT_SWITCH_OFF_CRUISE_CONTROL', 1: 'ESP_CCSWITCHOFF_SWITCH_OFF_CRUISE_CONTROL'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 8, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_status_0x243_243::Esp_ccswitchoffType esp_ccswitchoff(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_HDCAvailable', 'enum': {0: 'ESP_HDCAVAILABLE_AVAILABLE', 1: 'ESP_HDCAVAILABLE_NOT_AVAILABLE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 9, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_status_0x243_243::Esp_hdcavailableType esp_hdcavailable(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_cx75_PROTOCOL_ESP_STATUS_0X243_243_H_
