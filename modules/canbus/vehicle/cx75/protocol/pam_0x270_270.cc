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

#include "modules/canbus/vehicle/cx75/protocol/pam_0x270_270.h"

#include "modules/drivers/canbus/common/byte.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

using ::jmc_auto::drivers::canbus::Byte;

const int32_t Pam0x270270::ID = 0x270;

// public
Pam0x270270::Pam0x270270() { Reset(); }

uint32_t Pam0x270270::GetPeriod() const {
  // TODO modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Pam0x270270::UpdateData(uint8_t* data) {
  set_p_pam_esp_target_gear_request(data, pam_esp_target_gear_request_);
  set_p_pam_brakefunctionmode(data, pam_brakefunctionmode_);
  set_p_stopstartinhibit_apa(data, stopstartinhibit_apa_);
  set_p_pam_apaf(data, pam_apaf_);
  set_p_pam_cmdepssts(data, pam_cmdepssts_);
  set_p_pam_sts(data, pam_sts_);
  set_p_pam_brakemodests(data, pam_brakemodests_);
  set_p_pam_failurebrakemode(data, pam_failurebrakemode_);
  set_p_pam_esp_speed_target(data, pam_esp_speed_target_);
  set_p_pam_esp_stop_distance(data, pam_esp_stop_distance_);
  set_p_rolling_counter_0x270(data, rolling_counter_0x270_);
  set_p_checksum_0x270(data, checksum_0x270_);
  set_p_pam_trgtepsstrgwhlang(data, pam_trgtepsstrgwhlang_);
}

void Pam0x270270::Reset() {
  // TODO you should check this manually
  pam_esp_target_gear_request_ = Pam_0x270_270::PAM_ESP_TARGET_GEAR_REQUEST_PARK;
  pam_brakefunctionmode_ = Pam_0x270_270::PAM_BRAKEFUNCTIONMODE_NO_ACTION;
  stopstartinhibit_apa_ = Pam_0x270_270::STOPSTARTINHIBIT_APA_FALSE;
  pam_apaf_ = Pam_0x270_270::PAM_APAF_NORMAL;
  //pam_cmdepssts_ = Pam_0x270_270::PAM_CMDEPSSTS_CONTROL_EPS_REQUEST;
  pam_cmdepssts_ = Pam_0x270_270::PAM_CMDEPSSTS_NO_REQUEST;
  pam_sts_ = Pam_0x270_270::PAM_STS_OFF;
  //pam_brakemodests_ = Pam_0x270_270::PAM_BRAKEMODESTS_INIT;
  pam_brakemodests_ = Pam_0x270_270::PAM_BRAKEMODESTS_MANEUVERFINISHED;
  pam_failurebrakemode_ = Pam_0x270_270::PAM_FAILUREBRAKEMODE_IDLE_NO_BRAKING;
  pam_esp_speed_target_ = 0;
  pam_esp_stop_distance_ = 0;
  rolling_counter_0x270_ = 0;
  checksum_0x270_ = 0;
  pam_trgtepsstrgwhlang_ = 0;
}

Pam0x270270* Pam0x270270::set_pam_esp_target_gear_request(
    Pam_0x270_270::Pam_esp_target_gear_requestType pam_esp_target_gear_request) {
  pam_esp_target_gear_request_ = pam_esp_target_gear_request;
  return this;
 }

// config detail: {'name': 'PAM_ESP_Target_Gear_Request', 'enum': {0: 'PAM_ESP_TARGET_GEAR_REQUEST_NO_REQUEST', 1: 'PAM_ESP_TARGET_GEAR_REQUEST_PARK', 2: 'PAM_ESP_TARGET_GEAR_REQUEST_REVERSE', 3: 'PAM_ESP_TARGET_GEAR_REQUEST_NEUTRAL', 4: 'PAM_ESP_TARGET_GEAR_REQUEST_DRIVE'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 18, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Pam0x270270::set_p_pam_esp_target_gear_request(uint8_t* data,
    Pam_0x270_270::Pam_esp_target_gear_requestType pam_esp_target_gear_request) {
  int x = pam_esp_target_gear_request;

  Byte to_set(data + 2);
  to_set.set_value(x, 0, 3);
}


Pam0x270270* Pam0x270270::set_pam_brakefunctionmode(
    Pam_0x270_270::Pam_brakefunctionmodeType pam_brakefunctionmode) {
  pam_brakefunctionmode_ = pam_brakefunctionmode;
  return this;
 }

// config detail: {'name': 'PAM_BrakeFunctionMode', 'enum': {0: 'PAM_BRAKEFUNCTIONMODE_NO_ACTION', 1: 'PAM_BRAKEFUNCTIONMODE_AUTOMATICPARK'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 19, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Pam0x270270::set_p_pam_brakefunctionmode(uint8_t* data,
    Pam_0x270_270::Pam_brakefunctionmodeType pam_brakefunctionmode) {
  int x = pam_brakefunctionmode;

  Byte to_set(data + 2);
  to_set.set_value(x, 3, 1);
}


Pam0x270270* Pam0x270270::set_stopstartinhibit_apa(
    Pam_0x270_270::Stopstartinhibit_apaType stopstartinhibit_apa) {
  stopstartinhibit_apa_ = stopstartinhibit_apa;
  return this;
 }

// config detail: {'name': 'StopStartInhibit_APA', 'enum': {0: 'STOPSTARTINHIBIT_APA_FALSE', 1: 'STOPSTARTINHIBIT_APA_TRUE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 20, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Pam0x270270::set_p_stopstartinhibit_apa(uint8_t* data,
    Pam_0x270_270::Stopstartinhibit_apaType stopstartinhibit_apa) {
  int x = stopstartinhibit_apa;

  Byte to_set(data + 2);
  to_set.set_value(x, 4, 1);
}


Pam0x270270* Pam0x270270::set_pam_apaf(
    Pam_0x270_270::Pam_apafType pam_apaf) {
  pam_apaf_ = pam_apaf;
  return this;
 }

// config detail: {'name': 'PAM_APAF', 'enum': {0: 'PAM_APAF_NORMAL', 1: 'PAM_APAF_APA_FAILURE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 21, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Pam0x270270::set_p_pam_apaf(uint8_t* data,
    Pam_0x270_270::Pam_apafType pam_apaf) {
  int x = pam_apaf;

  Byte to_set(data + 2);
  to_set.set_value(x, 5, 1);
}


Pam0x270270* Pam0x270270::set_pam_cmdepssts(
    Pam_0x270_270::Pam_cmdepsstsType pam_cmdepssts) {
  pam_cmdepssts_ = pam_cmdepssts;
  return this;
 }

// config detail: {'name': 'PAM_CmdEPSSts', 'enum': {0: 'PAM_CMDEPSSTS_NO_REQUEST', 1: 'PAM_CMDEPSSTS_CONTROL_EPS_REQUEST', 2: 'PAM_CMDEPSSTS_EPS_CONTROL'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|2]', 'bit': 23, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Pam0x270270::set_p_pam_cmdepssts(uint8_t* data,
    Pam_0x270_270::Pam_cmdepsstsType pam_cmdepssts) {
  int x = pam_cmdepssts;

  Byte to_set(data + 2);
  to_set.set_value(x, 6, 2);
}


Pam0x270270* Pam0x270270::set_pam_sts(
    Pam_0x270_270::Pam_stsType pam_sts) {
  pam_sts_ = pam_sts;
  return this;
 }

// config detail: {'name': 'PAM_Sts', 'enum': {0: 'PAM_STS_OFF', 1: 'PAM_STS_STANDBY_STANDBY_ENABLE', 2: 'PAM_STS_SEARCHING_ENABLE', 3: 'PAM_STS_GUIDANCE_ACTIVE_ACTIVE', 4: 'PAM_STS_COMPLETED', 5: 'PAM_STS_FAILURE_DISABLE', 6: 'PAM_STS_TERMINATED', 7: 'PAM_STS_RESERVED'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 26, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Pam0x270270::set_p_pam_sts(uint8_t* data,
    Pam_0x270_270::Pam_stsType pam_sts) {
  int x = pam_sts;

  Byte to_set(data + 3);
  to_set.set_value(x, 0, 3);
}


Pam0x270270* Pam0x270270::set_pam_brakemodests(
    Pam_0x270_270::Pam_brakemodestsType pam_brakemodests) {
  pam_brakemodests_ = pam_brakemodests;
  return this;
 }

// config detail: {'name': 'PAM_BrakeModeSts', 'enum': {0: 'PAM_BRAKEMODESTS_INIT', 1: 'PAM_BRAKEMODESTS_STANDBY', 2: 'PAM_BRAKEMODESTS_ACTIVE', 3: 'PAM_BRAKEMODESTS_MANEUVERFINISHED', 4: 'PAM_BRAKEMODESTS_SUSPEND', 5: 'PAM_BRAKEMODESTS_ABORT'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 29, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Pam0x270270::set_p_pam_brakemodests(uint8_t* data,
    Pam_0x270_270::Pam_brakemodestsType pam_brakemodests) {
  int x = pam_brakemodests;

  Byte to_set(data + 3);
  to_set.set_value(x, 3, 3);
}


Pam0x270270* Pam0x270270::set_pam_failurebrakemode(
    Pam_0x270_270::Pam_failurebrakemodeType pam_failurebrakemode) {
  pam_failurebrakemode_ = pam_failurebrakemode;
  return this;
 }

// config detail: {'name': 'PAM_FailureBrakeMode', 'enum': {0: 'PAM_FAILUREBRAKEMODE_IDLE_NO_BRAKING', 1: 'PAM_FAILUREBRAKEMODE_COMFORTABLE_RESERVED', 2: 'PAM_FAILUREBRAKEMODE_EMERGENCY', 3: 'PAM_FAILUREBRAKEMODE_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 31, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Pam0x270270::set_p_pam_failurebrakemode(uint8_t* data,
    Pam_0x270_270::Pam_failurebrakemodeType pam_failurebrakemode) {
  int x = pam_failurebrakemode;

  Byte to_set(data + 3);
  to_set.set_value(x, 6, 2);
}


Pam0x270270* Pam0x270270::set_pam_esp_speed_target(
    float pam_esp_speed_target) {
  pam_esp_speed_target_ = pam_esp_speed_target;
  return this;
 }

// config detail: {'name': 'PAM_ESP_Speed_Target', 'enum': {255: 'PAM_ESP_SPEED_TARGET_INVALID'}, 'precision': 0.1, 'len': 8, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|25.4]', 'bit': 39, 'type': 'enum', 'order': 'motorola', 'physical_unit': 'kph'}
void Pam0x270270::set_p_pam_esp_speed_target(uint8_t* data,
    float pam_esp_speed_target) {
  int x = pam_esp_speed_target / 0.100000;

  Byte to_set(data + 4);
  to_set.set_value(x, 0, 8);
}


Pam0x270270* Pam0x270270::set_pam_esp_stop_distance(
    int pam_esp_stop_distance) {
  pam_esp_stop_distance_ = pam_esp_stop_distance;
  return this;
 }

// config detail: {'name': 'PAM_ESP_Stop_Distance', 'enum': {4095: 'PAM_ESP_STOP_DISTANCE_INVALID'}, 'precision': 1.0, 'len': 12, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|4094]', 'bit': 47, 'type': 'enum', 'order': 'motorola', 'physical_unit': 'cm'}
void Pam0x270270::set_p_pam_esp_stop_distance(uint8_t* data,
    int pam_esp_stop_distance) {
  int x = pam_esp_stop_distance;
  uint8_t t = 0;

  t = x & 0xF;
  Byte to_set0(data + 6);
  to_set0.set_value(t, 4, 4);
  x >>= 4;

  t = x & 0xFF;
  Byte to_set1(data + 5);
  to_set1.set_value(t, 0, 8);
}


Pam0x270270* Pam0x270270::set_rolling_counter_0x270(
    int rolling_counter_0x270) {
  rolling_counter_0x270_ = rolling_counter_0x270;
  return this;
 }

// config detail: {'name': 'Rolling_counter_0x270', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
void Pam0x270270::set_p_rolling_counter_0x270(uint8_t* data,
    int rolling_counter_0x270) {
  rolling_counter_0x270_++;
  rolling_counter_0x270_=rolling_counter_0x270_<=15?rolling_counter_0x270_:0;
  rolling_counter_0x270_ = ProtocolData::BoundedValue(0, 15, rolling_counter_0x270_);
  int x = rolling_counter_0x270_;

  Byte to_set(data + 6);
  to_set.set_value(x, 0, 4);
}


Pam0x270270* Pam0x270270::set_checksum_0x270(
    int checksum_0x270) {
  checksum_0x270_ = checksum_0x270;
  return this;
 }

// config detail: {'name': 'Checksum_0x270', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
void Pam0x270270::set_p_checksum_0x270(uint8_t* data,
    int checksum_0x270) {
  checksum_0x270 = (data[0]^data[1]^data[2]^data[3]^data[4]^data[5]^data[6]);
  checksum_0x270 = ProtocolData::BoundedValue(0, 255, checksum_0x270);
  int x = checksum_0x270;

  Byte to_set(data + 7);
  to_set.set_value(x, 0, 8);
}


Pam0x270270* Pam0x270270::set_pam_trgtepsstrgwhlang(
    double pam_trgtepsstrgwhlang) {
  pam_trgtepsstrgwhlang_ = pam_trgtepsstrgwhlang;
  return this;
 }

// config detail: {'name': 'PAM_TrgtEPSStrgWhlAng', 'enum': {65535: 'PAM_TRGTEPSSTRGWHLANG_INVALID'}, 'precision': 0.0238, 'len': 16, 'is_signed_var': False, 'offset': -780.0, 'physical_range': '[-780|779.7092]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': 'deg'}
void Pam0x270270::set_p_pam_trgtepsstrgwhlang(uint8_t* data,
    double pam_trgtepsstrgwhlang) {
  int x = (pam_trgtepsstrgwhlang - -780.000000) / 0.023800;
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(data + 1);
  to_set0.set_value(t, 0, 8);
  x >>= 8;

  t = x & 0xFF;
  Byte to_set1(data + 0);
  to_set1.set_value(t, 0, 8);
}

}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto
