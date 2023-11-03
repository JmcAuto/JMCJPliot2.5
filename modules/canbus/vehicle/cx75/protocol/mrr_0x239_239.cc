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

#include "modules/canbus/vehicle/cx75/protocol/mrr_0x239_239.h"

#include "modules/drivers/canbus/common/byte.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

using ::jmc_auto::drivers::canbus::Byte;

const int32_t Mrr0x239239::ID = 0x239;

// public
Mrr0x239239::Mrr0x239239() { Reset(); }

uint32_t Mrr0x239239::GetPeriod() const {
  // TODO modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Mrr0x239239::UpdateData(uint8_t* data) {
  set_p_acc_uppercomftbandreq(data, acc_uppercomftbandreq_);
  set_p_acc_brakepreferred(data, acc_brakepreferred_);
  set_p_eba_req(data, eba_req_);
  set_p_aeb_req(data, aeb_req_);
  set_p_acc_standstillreq(data, acc_standstillreq_);
  set_p_acc_driveoff(data, acc_driveoff_);
  set_p_awb_level(data, awb_level_);
  set_p_abp_req(data, abp_req_);
  set_p_awb_req(data, awb_req_);
  set_p_aba_req(data, aba_req_);
  set_p_aeb_tgtax(data, aeb_tgtax_);
  set_p_acc_state(data, acc_state_);
  set_p_rolling_counter_0x239(data, rolling_counter_0x239_);
  set_p_shutdownmode(data, shutdownmode_);
  set_p_aba_level(data, aba_level_);
  set_p_checksum_0x239(data, checksum_0x239_);
}

void Mrr0x239239::Reset() {
  // TODO you should check this manually
  acc_uppercomftbandreq_ = Mrr_0x239_239::ACC_UPPERCOMFTBANDREQ_NO_DEMAND;
  acc_brakepreferred_ = Mrr_0x239_239::ACC_BRAKEPREFERRED_NO_DEMAND;
  eba_req_ = Mrr_0x239_239::EBA_REQ_NO_DEMAND;
  aeb_req_ = Mrr_0x239_239::AEB_REQ_NO_DEMAND;
  acc_standstillreq_ = Mrr_0x239_239::ACC_STANDSTILLREQ_NO_DEMAND;
  acc_driveoff_ = Mrr_0x239_239::ACC_DRIVEOFF_NO_DEMAND;
  awb_level_ = Mrr_0x239_239::AWB_LEVEL_NO_LEVEL;
  abp_req_ = Mrr_0x239_239::ABP_REQ_NO_DEMAND;
  awb_req_ = Mrr_0x239_239::AWB_REQ_NO_DEMAND;
  aba_req_ = Mrr_0x239_239::ABA_REQ_NO_DEMAND;
  aeb_tgtax_ = 0.0;
  acc_state_ = Mrr_0x239_239::ACC_STATE_OFF_MODE;
  rolling_counter_0x239_ = 0;
  checksum_0x239_ = 0;
  shutdownmode_ = Mrr_0x239_239::SHUTDOWNMODE_SOFT_OFF;
  aba_level_ = Mrr_0x239_239::ABA_LEVEL_LEVEL_0;
}

void Mrr0x239239::checksum_rolling() {
  // TODO you should check this manually
  rolling_counter_0x239_++;
  rolling_counter_0x239_=rolling_counter_0x239_<=15?rolling_counter_0x239_:0;
  rolling_counter_0x239_ = ProtocolData::BoundedValue(0, 15, rolling_counter_0x239_);
}

Mrr0x239239* Mrr0x239239::set_acc_uppercomftbandreq(
    Mrr_0x239_239::Acc_uppercomftbandreqType acc_uppercomftbandreq) {
  acc_uppercomftbandreq_ = acc_uppercomftbandreq;
  return this;
 }

// config detail: {'name': 'ACC_UpperComftBandReq', 'enum': {0: 'ACC_UPPERCOMFTBANDREQ_NO_DEMAND', 1: 'ACC_UPPERCOMFTBANDREQ_DEMAND'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 0, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Mrr0x239239::set_p_acc_uppercomftbandreq(uint8_t* data,
    Mrr_0x239_239::Acc_uppercomftbandreqType acc_uppercomftbandreq) {
  int x = acc_uppercomftbandreq;

  Byte to_set(data + 0);
  to_set.set_value(x, 0, 1);
}


Mrr0x239239* Mrr0x239239::set_acc_brakepreferred(
    Mrr_0x239_239::Acc_brakepreferredType acc_brakepreferred) {
  acc_brakepreferred_ = acc_brakepreferred;
  return this;
 }

// config detail: {'name': 'ACC_BrakePreferred', 'enum': {0: 'ACC_BRAKEPREFERRED_NO_DEMAND', 1: 'ACC_BRAKEPREFERRED_DEMAND'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 1, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Mrr0x239239::set_p_acc_brakepreferred(uint8_t* data,
    Mrr_0x239_239::Acc_brakepreferredType acc_brakepreferred) {
  int x = acc_brakepreferred;

  Byte to_set(data + 0);
  to_set.set_value(x, 1, 1);
}


Mrr0x239239* Mrr0x239239::set_eba_req(
    Mrr_0x239_239::Eba_reqType eba_req) {
  eba_req_ = eba_req;
  return this;
 }

// config detail: {'name': 'EBA_Req', 'enum': {0: 'EBA_REQ_NO_DEMAND', 1: 'EBA_REQ_DEMAND'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 12, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Mrr0x239239::set_p_eba_req(uint8_t* data,
    Mrr_0x239_239::Eba_reqType eba_req) {
  int x = eba_req;

  Byte to_set(data + 1);
  to_set.set_value(x, 4, 1);
}


Mrr0x239239* Mrr0x239239::set_aeb_req(
    Mrr_0x239_239::Aeb_reqType aeb_req) {
  aeb_req_ = aeb_req;
  return this;
 }

// config detail: {'name': 'AEB_Req', 'enum': {0: 'AEB_REQ_NO_DEMAND', 1: 'AEB_REQ_DEMAND'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 13, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Mrr0x239239::set_p_aeb_req(uint8_t* data,
    Mrr_0x239_239::Aeb_reqType aeb_req) {
  int x = aeb_req;

  Byte to_set(data + 1);
  to_set.set_value(x, 5, 1);
}


Mrr0x239239* Mrr0x239239::set_acc_standstillreq(
    Mrr_0x239_239::Acc_standstillreqType acc_standstillreq) {
  acc_standstillreq_ = acc_standstillreq;
  return this;
 }

// config detail: {'name': 'ACC_StandstillReq', 'enum': {0: 'ACC_STANDSTILLREQ_NO_DEMAND', 1: 'ACC_STANDSTILLREQ_DEMAND'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 14, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Mrr0x239239::set_p_acc_standstillreq(uint8_t* data,
    Mrr_0x239_239::Acc_standstillreqType acc_standstillreq) {
  int x = acc_standstillreq;

  Byte to_set(data + 1);
  to_set.set_value(x, 6, 1);
}


Mrr0x239239* Mrr0x239239::set_acc_driveoff(
    Mrr_0x239_239::Acc_driveoffType acc_driveoff) {
  acc_driveoff_ = acc_driveoff;
  return this;
 }

// config detail: {'name': 'ACC_DriveOff', 'enum': {0: 'ACC_DRIVEOFF_NO_DEMAND', 1: 'ACC_DRIVEOFF_DEMAND'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 15, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Mrr0x239239::set_p_acc_driveoff(uint8_t* data,
    Mrr_0x239_239::Acc_driveoffType acc_driveoff) {
  int x = acc_driveoff;

  Byte to_set(data + 1);
  to_set.set_value(x, 7, 1);
}


Mrr0x239239* Mrr0x239239::set_awb_level(
    Mrr_0x239_239::Awb_levelType awb_level) {
  awb_level_ = awb_level;
  return this;
 }

// config detail: {'name': 'AWB_Level', 'enum': {0: 'AWB_LEVEL_NO_LEVEL', 1: 'AWB_LEVEL_LEVEL_1', 2: 'AWB_LEVEL_LEVEL_2', 3: 'AWB_LEVEL_LEVEL_3', 4: 'AWB_LEVEL_LEVEL_4'}, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|15]', 'bit': 19, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Mrr0x239239::set_p_awb_level(uint8_t* data,
    Mrr_0x239_239::Awb_levelType awb_level) {
  int x = awb_level;

  Byte to_set(data + 2);
  to_set.set_value(x, 0, 4);
}


Mrr0x239239* Mrr0x239239::set_abp_req(
    Mrr_0x239_239::Abp_reqType abp_req) {
  abp_req_ = abp_req;
  return this;
 }

// config detail: {'name': 'ABP_Req', 'enum': {0: 'ABP_REQ_NO_DEMAND', 1: 'ABP_REQ_DEMAND'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 21, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Mrr0x239239::set_p_abp_req(uint8_t* data,
    Mrr_0x239_239::Abp_reqType abp_req) {
  int x = abp_req;

  Byte to_set(data + 2);
  to_set.set_value(x, 5, 1);
}


Mrr0x239239* Mrr0x239239::set_awb_req(
    Mrr_0x239_239::Awb_reqType awb_req) {
  awb_req_ = awb_req;
  return this;
 }

// config detail: {'name': 'AWB_Req', 'enum': {0: 'AWB_REQ_NO_DEMAND', 1: 'AWB_REQ_DEMAND'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 22, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Mrr0x239239::set_p_awb_req(uint8_t* data,
    Mrr_0x239_239::Awb_reqType awb_req) {
  int x = awb_req;

  Byte to_set(data + 2);
  to_set.set_value(x, 6, 1);
}


Mrr0x239239* Mrr0x239239::set_aba_req(
    Mrr_0x239_239::Aba_reqType aba_req) {
  aba_req_ = aba_req;
  return this;
 }

// config detail: {'name': 'ABA_Req', 'enum': {0: 'ABA_REQ_NO_DEMAND', 1: 'ABA_REQ_DEMAND'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 23, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Mrr0x239239::set_p_aba_req(uint8_t* data,
    Mrr_0x239_239::Aba_reqType aba_req) {
  int x = aba_req;

  Byte to_set(data + 2);
  to_set.set_value(x, 7, 1);
}


Mrr0x239239* Mrr0x239239::set_aeb_tgtax(
    double aeb_tgtax) {
  aeb_tgtax_ = aeb_tgtax;
  return this;
 }

// config detail: {'name': 'AEB_TgtAx', 'offset': -16.0, 'precision': 0.05, 'len': 16, 'is_signed_var': False, 'physical_range': '[-16|16]', 'bit': 31, 'type': 'double', 'order': 'motorola', 'physical_unit': 'm s2'}
void Mrr0x239239::set_p_aeb_tgtax(uint8_t* data,
    double aeb_tgtax) {
  aeb_tgtax = ProtocolData::BoundedValue(-16.0, 16.0, aeb_tgtax);
  int x = (aeb_tgtax - -16.000000) / 0.050000;
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(data + 4);
  to_set0.set_value(t, 0, 8);
  x >>= 8;

  t = x & 0xFF;
  Byte to_set1(data + 3);
  to_set1.set_value(t, 0, 8);
}


Mrr0x239239* Mrr0x239239::set_acc_state(
    Mrr_0x239_239::Acc_stateType acc_state) {
  acc_state_ = acc_state;
  return this;
 }

// config detail: {'name': 'ACC_State', 'enum': {0: 'ACC_STATE_OFF_MODE', 1: 'ACC_STATE_PASSIVE_MODE', 2: 'ACC_STATE_STAND_BY_MODE', 3: 'ACC_STATE_ACTIVE_CONTROL_MODE', 4: 'ACC_STATE_BRAKE_ONLY_MODE', 5: 'ACC_STATE_OVERRIDE', 6: 'ACC_STATE_STANDSTILL', 7: 'ACC_STATE_FAILURE_MODE'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 5, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Mrr0x239239::set_p_acc_state(uint8_t* data,
    Mrr_0x239_239::Acc_stateType acc_state) {
  int x = acc_state;

  Byte to_set(data + 0);
  to_set.set_value(x, 3, 3);
}


Mrr0x239239* Mrr0x239239::set_rolling_counter_0x239(
    int rolling_counter_0x239) {
  rolling_counter_0x239_ = rolling_counter_0x239;
  return this;
 }

// config detail: {'name': 'Rolling_counter_0x239', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
void Mrr0x239239::set_p_rolling_counter_0x239(uint8_t* data,
    int rolling_counter_0x239) {
  //rolling_counter_0x239_++;
  //rolling_counter_0x239_=rolling_counter_0x239_<=15?rolling_counter_0x239_:0;
  //rolling_counter_0x239_ = ProtocolData::BoundedValue(0, 15, rolling_counter_0x239_);
  //int x = rolling_counter_0x239_;

  //Byte to_set(data + 6);
  //to_set.set_value(x, 0, 4);
}


Mrr0x239239* Mrr0x239239::set_checksum_0x239(
    int checksum_0x239) {
  checksum_0x239_ = checksum_0x239;
  return this;
 }

// config detail: {'name': 'Checksum_0x239', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
void Mrr0x239239::set_p_checksum_0x239(uint8_t* data,
    int checksum_0x239) {
  //checksum_0x239 = (data[0]^data[1]^data[2]^data[3]^data[4]^data[5]^data[6]);
  //checksum_0x239 = ProtocolData::BoundedValue(0, 255, checksum_0x239);
  //int x = checksum_0x239;

  //Byte to_set(data + 7);
  //to_set.set_value(x, 0, 8);
}


Mrr0x239239* Mrr0x239239::set_shutdownmode(
    Mrr_0x239_239::ShutdownmodeType shutdownmode) {
  shutdownmode_ = shutdownmode;
  return this;
 }

// config detail: {'name': 'ShutdownMode', 'enum': {0: 'SHUTDOWNMODE_SOFT_OFF', 1: 'SHUTDOWNMODE_FAST_OFF', 2: 'SHUTDOWNMODE_RESERVED', 3: 'SHUTDOWNMODE_INITIAL'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Mrr0x239239::set_p_shutdownmode(uint8_t* data,
    Mrr_0x239_239::ShutdownmodeType shutdownmode) {
  int x = shutdownmode;

  Byte to_set(data + 0);
  to_set.set_value(x, 6, 2);
}


Mrr0x239239* Mrr0x239239::set_aba_level(
    Mrr_0x239_239::Aba_levelType aba_level) {
  aba_level_ = aba_level;
  return this;
 }

// config detail: {'name': 'ABA_Level', 'enum': {0: 'ABA_LEVEL_LEVEL_0', 1: 'ABA_LEVEL_LEVEL_1', 2: 'ABA_LEVEL_LEVEL_2', 3: 'ABA_LEVEL_LEVEL_3'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 9, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Mrr0x239239::set_p_aba_level(uint8_t* data,
    Mrr_0x239_239::Aba_levelType aba_level) {
  int x = aba_level;

  Byte to_set(data + 1);
  to_set.set_value(x, 0, 2);
}

}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto
