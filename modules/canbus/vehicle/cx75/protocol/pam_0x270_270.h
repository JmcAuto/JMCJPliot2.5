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

#ifndef MODULES_CANBUS_VEHICLE_cx75_PROTOCOL_PAM_0X270_270_H_
#define MODULES_CANBUS_VEHICLE_cx75_PROTOCOL_PAM_0X270_270_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

class Pam0x270270 : public ::jmc_auto::drivers::canbus::ProtocolData<
                    ::jmc_auto::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Pam0x270270();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'name': 'PAM_ESP_Target_Gear_Request', 'enum': {0: 'PAM_ESP_TARGET_GEAR_REQUEST_NO_REQUEST', 1: 'PAM_ESP_TARGET_GEAR_REQUEST_PARK', 2: 'PAM_ESP_TARGET_GEAR_REQUEST_REVERSE', 3: 'PAM_ESP_TARGET_GEAR_REQUEST_NEUTRAL', 4: 'PAM_ESP_TARGET_GEAR_REQUEST_DRIVE'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 18, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Pam0x270270* set_pam_esp_target_gear_request(Pam_0x270_270::Pam_esp_target_gear_requestType pam_esp_target_gear_request);

  // config detail: {'name': 'PAM_BrakeFunctionMode', 'enum': {0: 'PAM_BRAKEFUNCTIONMODE_NO_ACTION', 1: 'PAM_BRAKEFUNCTIONMODE_AUTOMATICPARK'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 19, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Pam0x270270* set_pam_brakefunctionmode(Pam_0x270_270::Pam_brakefunctionmodeType pam_brakefunctionmode);

  // config detail: {'name': 'StopStartInhibit_APA', 'enum': {0: 'STOPSTARTINHIBIT_APA_FALSE', 1: 'STOPSTARTINHIBIT_APA_TRUE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 20, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Pam0x270270* set_stopstartinhibit_apa(Pam_0x270_270::Stopstartinhibit_apaType stopstartinhibit_apa);

  // config detail: {'name': 'PAM_APAF', 'enum': {0: 'PAM_APAF_NORMAL', 1: 'PAM_APAF_APA_FAILURE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 21, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Pam0x270270* set_pam_apaf(Pam_0x270_270::Pam_apafType pam_apaf);

  // config detail: {'name': 'PAM_CmdEPSSts', 'enum': {0: 'PAM_CMDEPSSTS_NO_REQUEST', 1: 'PAM_CMDEPSSTS_CONTROL_EPS_REQUEST', 2: 'PAM_CMDEPSSTS_EPS_CONTROL'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|2]', 'bit': 23, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Pam0x270270* set_pam_cmdepssts(Pam_0x270_270::Pam_cmdepsstsType pam_cmdepssts);

  // config detail: {'name': 'PAM_Sts', 'enum': {0: 'PAM_STS_OFF', 1: 'PAM_STS_STANDBY_STANDBY_ENABLE', 2: 'PAM_STS_SEARCHING_ENABLE', 3: 'PAM_STS_GUIDANCE_ACTIVE_ACTIVE', 4: 'PAM_STS_COMPLETED', 5: 'PAM_STS_FAILURE_DISABLE', 6: 'PAM_STS_TERMINATED', 7: 'PAM_STS_RESERVED'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 26, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Pam0x270270* set_pam_sts(Pam_0x270_270::Pam_stsType pam_sts);

  // config detail: {'name': 'PAM_BrakeModeSts', 'enum': {0: 'PAM_BRAKEMODESTS_INIT', 1: 'PAM_BRAKEMODESTS_STANDBY', 2: 'PAM_BRAKEMODESTS_ACTIVE', 3: 'PAM_BRAKEMODESTS_MANEUVERFINISHED', 4: 'PAM_BRAKEMODESTS_SUSPEND', 5: 'PAM_BRAKEMODESTS_ABORT'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 29, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Pam0x270270* set_pam_brakemodests(Pam_0x270_270::Pam_brakemodestsType pam_brakemodests);

  // config detail: {'name': 'PAM_FailureBrakeMode', 'enum': {0: 'PAM_FAILUREBRAKEMODE_IDLE_NO_BRAKING', 1: 'PAM_FAILUREBRAKEMODE_COMFORTABLE_RESERVED', 2: 'PAM_FAILUREBRAKEMODE_EMERGENCY', 3: 'PAM_FAILUREBRAKEMODE_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 31, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Pam0x270270* set_pam_failurebrakemode(Pam_0x270_270::Pam_failurebrakemodeType pam_failurebrakemode);

  // config detail: {'name': 'PAM_ESP_Speed_Target', 'enum': {255: 'PAM_ESP_SPEED_TARGET_INVALID'}, 'precision': 0.1, 'len': 8, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|25.4]', 'bit': 39, 'type': 'enum', 'order': 'motorola', 'physical_unit': 'kph'}
  Pam0x270270* set_pam_esp_speed_target(float pam_esp_speed_target);

  // config detail: {'name': 'PAM_ESP_Stop_Distance', 'enum': {4095: 'PAM_ESP_STOP_DISTANCE_INVALID'}, 'precision': 1.0, 'len': 12, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|4094]', 'bit': 47, 'type': 'enum', 'order': 'motorola', 'physical_unit': 'cm'}
  Pam0x270270* set_pam_esp_stop_distance(int pam_esp_stop_distance);

  // config detail: {'name': 'Rolling_counter_0x270', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  Pam0x270270* set_rolling_counter_0x270(int rolling_counter_0x270);

  // config detail: {'name': 'Checksum_0x270', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  Pam0x270270* set_checksum_0x270(int checksum_0x270);

  // config detail: {'name': 'PAM_TrgtEPSStrgWhlAng', 'enum': {65535: 'PAM_TRGTEPSSTRGWHLANG_INVALID'}, 'precision': 0.0238, 'len': 16, 'is_signed_var': False, 'offset': -780.0, 'physical_range': '[-780|779.7092]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': 'deg'}
  Pam0x270270* set_pam_trgtepsstrgwhlang(double pam_trgtepsstrgwhlang);

 private:

  // config detail: {'name': 'PAM_ESP_Target_Gear_Request', 'enum': {0: 'PAM_ESP_TARGET_GEAR_REQUEST_NO_REQUEST', 1: 'PAM_ESP_TARGET_GEAR_REQUEST_PARK', 2: 'PAM_ESP_TARGET_GEAR_REQUEST_REVERSE', 3: 'PAM_ESP_TARGET_GEAR_REQUEST_NEUTRAL', 4: 'PAM_ESP_TARGET_GEAR_REQUEST_DRIVE'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 18, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_pam_esp_target_gear_request(uint8_t* data, Pam_0x270_270::Pam_esp_target_gear_requestType pam_esp_target_gear_request);

  // config detail: {'name': 'PAM_BrakeFunctionMode', 'enum': {0: 'PAM_BRAKEFUNCTIONMODE_NO_ACTION', 1: 'PAM_BRAKEFUNCTIONMODE_AUTOMATICPARK'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 19, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_pam_brakefunctionmode(uint8_t* data, Pam_0x270_270::Pam_brakefunctionmodeType pam_brakefunctionmode);

  // config detail: {'name': 'StopStartInhibit_APA', 'enum': {0: 'STOPSTARTINHIBIT_APA_FALSE', 1: 'STOPSTARTINHIBIT_APA_TRUE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 20, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_stopstartinhibit_apa(uint8_t* data, Pam_0x270_270::Stopstartinhibit_apaType stopstartinhibit_apa);

  // config detail: {'name': 'PAM_APAF', 'enum': {0: 'PAM_APAF_NORMAL', 1: 'PAM_APAF_APA_FAILURE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 21, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_pam_apaf(uint8_t* data, Pam_0x270_270::Pam_apafType pam_apaf);

  // config detail: {'name': 'PAM_CmdEPSSts', 'enum': {0: 'PAM_CMDEPSSTS_NO_REQUEST', 1: 'PAM_CMDEPSSTS_CONTROL_EPS_REQUEST', 2: 'PAM_CMDEPSSTS_EPS_CONTROL'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|2]', 'bit': 23, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_pam_cmdepssts(uint8_t* data, Pam_0x270_270::Pam_cmdepsstsType pam_cmdepssts);

  // config detail: {'name': 'PAM_Sts', 'enum': {0: 'PAM_STS_OFF', 1: 'PAM_STS_STANDBY_STANDBY_ENABLE', 2: 'PAM_STS_SEARCHING_ENABLE', 3: 'PAM_STS_GUIDANCE_ACTIVE_ACTIVE', 4: 'PAM_STS_COMPLETED', 5: 'PAM_STS_FAILURE_DISABLE', 6: 'PAM_STS_TERMINATED', 7: 'PAM_STS_RESERVED'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 26, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_pam_sts(uint8_t* data, Pam_0x270_270::Pam_stsType pam_sts);

  // config detail: {'name': 'PAM_BrakeModeSts', 'enum': {0: 'PAM_BRAKEMODESTS_INIT', 1: 'PAM_BRAKEMODESTS_STANDBY', 2: 'PAM_BRAKEMODESTS_ACTIVE', 3: 'PAM_BRAKEMODESTS_MANEUVERFINISHED', 4: 'PAM_BRAKEMODESTS_SUSPEND', 5: 'PAM_BRAKEMODESTS_ABORT'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 29, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_pam_brakemodests(uint8_t* data, Pam_0x270_270::Pam_brakemodestsType pam_brakemodests);

  // config detail: {'name': 'PAM_FailureBrakeMode', 'enum': {0: 'PAM_FAILUREBRAKEMODE_IDLE_NO_BRAKING', 1: 'PAM_FAILUREBRAKEMODE_COMFORTABLE_RESERVED', 2: 'PAM_FAILUREBRAKEMODE_EMERGENCY', 3: 'PAM_FAILUREBRAKEMODE_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 31, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_pam_failurebrakemode(uint8_t* data, Pam_0x270_270::Pam_failurebrakemodeType pam_failurebrakemode);

  // config detail: {'name': 'PAM_ESP_Speed_Target', 'enum': {255: 'PAM_ESP_SPEED_TARGET_INVALID'}, 'precision': 0.1, 'len': 8, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|25.4]', 'bit': 39, 'type': 'enum', 'order': 'motorola', 'physical_unit': 'kph'}
  void set_p_pam_esp_speed_target(uint8_t* data, float pam_esp_speed_target);

  // config detail: {'name': 'PAM_ESP_Stop_Distance', 'enum': {4095: 'PAM_ESP_STOP_DISTANCE_INVALID'}, 'precision': 1.0, 'len': 12, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|4094]', 'bit': 47, 'type': 'enum', 'order': 'motorola', 'physical_unit': 'cm'}
  void set_p_pam_esp_stop_distance(uint8_t* data, int pam_esp_stop_distance);

  // config detail: {'name': 'Rolling_counter_0x270', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  void set_p_rolling_counter_0x270(uint8_t* data, int rolling_counter_0x270);

  // config detail: {'name': 'Checksum_0x270', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  void set_p_checksum_0x270(uint8_t* data, int checksum_0x270);

  // config detail: {'name': 'PAM_TrgtEPSStrgWhlAng', 'enum': {65535: 'PAM_TRGTEPSSTRGWHLANG_INVALID'}, 'precision': 0.0238, 'len': 16, 'is_signed_var': False, 'offset': -780.0, 'physical_range': '[-780|779.7092]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': 'deg'}
  void set_p_pam_trgtepsstrgwhlang(uint8_t* data, double pam_trgtepsstrgwhlang);

 private:
  Pam_0x270_270::Pam_esp_target_gear_requestType pam_esp_target_gear_request_;
  Pam_0x270_270::Pam_brakefunctionmodeType pam_brakefunctionmode_;
  Pam_0x270_270::Stopstartinhibit_apaType stopstartinhibit_apa_;
  Pam_0x270_270::Pam_apafType pam_apaf_;
  Pam_0x270_270::Pam_cmdepsstsType pam_cmdepssts_;
  Pam_0x270_270::Pam_stsType pam_sts_;
  Pam_0x270_270::Pam_brakemodestsType pam_brakemodests_;
  Pam_0x270_270::Pam_failurebrakemodeType pam_failurebrakemode_;
  float pam_esp_speed_target_;
  int pam_esp_stop_distance_;
  int rolling_counter_0x270_;
  int checksum_0x270_;
  double pam_trgtepsstrgwhlang_;
};

}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_cx75_PROTOCOL_PAM_0X270_270_H_
