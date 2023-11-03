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

#ifndef MODULES_CANBUS_VEHICLE_cx75_PROTOCOL_ESP_VLC_0X223_223_H_
#define MODULES_CANBUS_VEHICLE_cx75_PROTOCOL_ESP_VLC_0X223_223_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

class Espvlc0x223223 : public ::jmc_auto::drivers::canbus::ProtocolData<
                    ::jmc_auto::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Espvlc0x223223();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'name': 'ESP_VLC_InternalTargetAcce', 'offset': -7.0, 'precision': 0.05, 'len': 8, 'is_signed_var': False, 'physical_range': '[-7|5.75]', 'bit': 23, 'type': 'double', 'order': 'motorola', 'physical_unit': ''}
  double esp_vlc_internaltargetacce(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_APA_GearBoxEnable', 'enum': {0: 'ESP_APA_GEARBOXENABLE_NO_REQUEST', 1: 'ESP_APA_GEARBOXENABLE_GEAR_SHIFT_REQUEST'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 28, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_vlc_0x223_223::Esp_apa_gearboxenableType esp_apa_gearboxenable(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_Target_Gear_Request', 'enum': {0: 'ESP_TARGET_GEAR_REQUEST_NO_REQUEST', 1: 'ESP_TARGET_GEAR_REQUEST_PARK', 2: 'ESP_TARGET_GEAR_REQUEST_REVERSE', 3: 'ESP_TARGET_GEAR_REQUEST_NEUTRAL', 4: 'ESP_TARGET_GEAR_REQUEST_DRIVE'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 31, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_vlc_0x223_223::Esp_target_gear_requestType esp_target_gear_request(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_VLC_APactive', 'enum': {0: 'ESP_VLC_APACTIVE_NOT_ACTIVE', 1: 'ESP_VLC_APACTIVE_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 32, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_vlc_0x223_223::Esp_vlc_apactiveType esp_vlc_apactive(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_PAM_LC_FailureSts', 'enum': {0: 'ESP_PAM_LC_FAILURESTS_NO_ERROR', 1: 'ESP_PAM_LC_FAILURESTS_VEHICLE_BLOCKED', 2: 'ESP_PAM_LC_FAILURESTS_UNEXPECTED_GEARPOSITION', 3: 'ESP_PAM_LC_FAILURESTS_UNEXPECTED_EPB_ACTION', 4: 'ESP_PAM_LC_FAILURESTS_UNEXPECTED_ACCPEDALINTERVENTION', 5: 'ESP_PAM_LC_FAILURESTS_UNEXPECTED_GEARINTERVENTION', 7: 'ESP_PAM_LC_FAILURESTS_ERROR'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 35, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_vlc_0x223_223::Esp_pam_lc_failurestsType esp_pam_lc_failurests(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_PAM_LC_Status', 'enum': {0: 'ESP_PAM_LC_STATUS_OFF', 1: 'ESP_PAM_LC_STATUS_STANDBY', 10: 'ESP_PAM_LC_STATUS_ERROR', 4: 'ESP_PAM_LC_STATUS_ACTIVE_AUTOMATICPARK'}, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|15]', 'bit': 39, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_vlc_0x223_223::Esp_pam_lc_statusType esp_pam_lc_status(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Rolling_counter_0x223', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int rolling_counter_0x223(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_VLC_Active', 'enum': {0: 'ESP_VLC_ACTIVE_NOT_ACTIVE', 1: 'ESP_VLC_ACTIVE_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 52, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_vlc_0x223_223::Esp_vlc_activeType esp_vlc_active(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_VLC_Available', 'enum': {0: 'ESP_VLC_AVAILABLE_NOT_AVAILABLE', 1: 'ESP_VLC_AVAILABLE_AVAILABLE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 53, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_vlc_0x223_223::Esp_vlc_availableType esp_vlc_available(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_VLCAPA_Available', 'enum': {0: 'ESP_VLCAPA_AVAILABLE_NOT_AVAILABLE', 1: 'ESP_VLCAPA_AVAILABLE_AVAILABLE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 54, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_vlc_0x223_223::Esp_vlcapa_availableType esp_vlcapa_available(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_VLCEngTorqReqAct', 'enum': {0: 'ESP_VLCENGTORQREQACT_VALID', 1: 'ESP_VLCENGTORQREQACT_INVALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 55, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_vlc_0x223_223::Esp_vlcengtorqreqactType esp_vlcengtorqreqact(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Checksum_0x223', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int checksum_0x223(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_VLCEngTorqReq', 'offset': -30000.0, 'precision': 1.0, 'len': 16, 'is_signed_var': False, 'physical_range': '[-30000|30000]', 'bit': 7, 'type': 'int', 'order': 'motorola', 'physical_unit': 'Nm'}
  int esp_vlcengtorqreq(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_cx75_PROTOCOL_ESP_VLC_0X223_223_H_
