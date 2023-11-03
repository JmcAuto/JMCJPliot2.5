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

#include "modules/canbus/vehicle/teshun/protocol/adu_controlbrake_0x110_110.h"

#include "modules/drivers/canbus/common/byte.h"

namespace jmc_auto {
namespace canbus {
namespace teshun {

using ::jmc_auto::drivers::canbus::Byte;

const int32_t Aducontrolbrake0x110110::ID = 0x110;

// public
Aducontrolbrake0x110110::Aducontrolbrake0x110110() { Reset(); }

uint32_t Aducontrolbrake0x110110::GetPeriod() const {
  // TODO modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Aducontrolbrake0x110110::UpdateData(uint8_t* data) {

  set_p_ic_rolling_counter_0x110(data, ic_rolling_counter_0x110_);
  set_p_adu_tgt_deceleration(data, adu_tgt_deceleration_);
  set_p_adu_brktmcposition_req(data, adu_brktmcposition_req_);
  set_p_adu_parkrelease_req(data, adu_parkrelease_req_);
  set_p_adu_controbrk_standstill(data, adu_controbrk_standstill_);
  set_p_adu_controbrk_enable(data, adu_controbrk_enable_);
  set_p_adu_mastercylinderpressreq(data, adu_mastercylinderpressreq_);
  set_p_ic_checksum_0x110(data, ic_checksum_0x110_);
}

void Aducontrolbrake0x110110::Reset() {
  // TODO you should check this manually
  ic_checksum_0x110_ = 0;
  ic_rolling_counter_0x110_ = 0;
  adu_tgt_deceleration_ = 0.0;
  adu_brktmcposition_req_ = 0.0;
  adu_parkrelease_req_ = Adu_controlbrake_0x110_110::ADU_PARKRELEASE_REQ_NO_CONTROL;
  adu_controbrk_standstill_ = Adu_controlbrake_0x110_110::ADU_CONTROBRK_STANDSTILL_NOT_STANDSTILL;
  adu_controbrk_enable_ = Adu_controlbrake_0x110_110::ADU_CONTROBRK_ENABLE_DISABLE;
  adu_mastercylinderpressreq_ = 0;
}

Aducontrolbrake0x110110* Aducontrolbrake0x110110::set_ic_checksum_0x110(
    int ic_checksum_0x110) {
  ic_checksum_0x110_ = ic_checksum_0x110;
  return this;
 }

// config detail: {'name': 'IC_Checksum_0x110', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
void Aducontrolbrake0x110110::set_p_ic_checksum_0x110(uint8_t* data,
    int ic_checksum_0x110) {

ic_checksum_0x110 = (data[0]^data[1]^data[2]^data[3]^data[4]^data[5]^data[6]);
  ic_checksum_0x110 = ProtocolData::BoundedValue(0, 255, ic_checksum_0x110);
  int x = ic_checksum_0x110;

  Byte to_set(data + 7);
  to_set.set_value(x, 0, 8);
}


Aducontrolbrake0x110110* Aducontrolbrake0x110110::set_ic_rolling_counter_0x110(
    int ic_rolling_counter_0x110) {
  ic_rolling_counter_0x110_ = ic_rolling_counter_0x110;
  return this;
 }

// config detail: {'name': 'IC_Rolling_counter_0x110', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
void Aducontrolbrake0x110110::set_p_ic_rolling_counter_0x110(uint8_t* data,
    int ic_rolling_counter_0x110) {
  ic_rolling_counter_0x110 = ProtocolData::BoundedValue(0, 15, ic_rolling_counter_0x110);
  int x = ic_rolling_counter_0x110;

  Byte to_set(data + 6);
  to_set.set_value(x, 0, 4);
}


Aducontrolbrake0x110110* Aducontrolbrake0x110110::set_adu_tgt_deceleration(
    double adu_tgt_deceleration) {
  adu_tgt_deceleration_ = adu_tgt_deceleration;
  return this;
 }

// config detail: {'name': 'ADU_Tgt_Deceleration', 'offset': 0.0, 'precision': 0.00159276, 'len': 12, 'is_signed_var': True, 'physical_range': '[0|0]', 'bit': 47, 'type': 'double', 'order': 'motorola', 'physical_unit': 'g'}
void Aducontrolbrake0x110110::set_p_adu_tgt_deceleration(uint8_t* data,
    double adu_tgt_deceleration) {
  adu_tgt_deceleration = ProtocolData::BoundedValue(0.0, 0.0, adu_tgt_deceleration);
  int x = adu_tgt_deceleration / 0.001593;
  uint8_t t = 0;

  t = x & 0xF;
  Byte to_set0(data + 6);
  to_set0.set_value(t, 4, 4);
  x >>= 4;

  t = x & 0xFF;
  Byte to_set1(data + 5);
  to_set1.set_value(t, 0, 8);
}


Aducontrolbrake0x110110* Aducontrolbrake0x110110::set_adu_brktmcposition_req(
    double adu_brktmcposition_req) {
  adu_brktmcposition_req_ = adu_brktmcposition_req;
  return this;
 }

// config detail: {'name': 'ADU_BrkTMCPosition_Req', 'offset': 0.0, 'precision': 0.1, 'len': 10, 'is_signed_var': False, 'physical_range': '[0|100]', 'bit': 31, 'type': 'double', 'order': 'motorola', 'physical_unit': '%'}
void Aducontrolbrake0x110110::set_p_adu_brktmcposition_req(uint8_t* data,
    double adu_brktmcposition_req) {
  adu_brktmcposition_req = ProtocolData::BoundedValue(0.0, 100.0, adu_brktmcposition_req);
  int x = adu_brktmcposition_req / 0.100000;
  uint8_t t = 0;

  t = x & 0x3;
  Byte to_set0(data + 4);
  to_set0.set_value(t, 6, 2);
  x >>= 2;

  t = x & 0xFF;
  Byte to_set1(data + 3);
  to_set1.set_value(t, 0, 8);
}


Aducontrolbrake0x110110* Aducontrolbrake0x110110::set_adu_parkrelease_req(
    Adu_controlbrake_0x110_110::Adu_parkrelease_reqType adu_parkrelease_req) {
  adu_parkrelease_req_ = adu_parkrelease_req;
  return this;
 }

// config detail: {'name': 'ADU_ParkRelease_Req', 'enum': {0: 'ADU_PARKRELEASE_REQ_NO_CONTROL', 1: 'ADU_PARKRELEASE_REQ_RELEASE', 2: 'ADU_PARKRELEASE_REQ_PARK', 3: 'ADU_PARKRELEASE_REQ_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 23, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Aducontrolbrake0x110110::set_p_adu_parkrelease_req(uint8_t* data,
    Adu_controlbrake_0x110_110::Adu_parkrelease_reqType adu_parkrelease_req) {
  int x = adu_parkrelease_req;

  Byte to_set(data + 2);
  to_set.set_value(x, 6, 2);
}


Aducontrolbrake0x110110* Aducontrolbrake0x110110::set_adu_controbrk_standstill(
    Adu_controlbrake_0x110_110::Adu_controbrk_standstillType adu_controbrk_standstill) {
  adu_controbrk_standstill_ = adu_controbrk_standstill;
  return this;
 }

// config detail: {'name': 'ADU_ControBrk_StandStill', 'enum': {0: 'ADU_CONTROBRK_STANDSTILL_NOT_STANDSTILL', 1: 'ADU_CONTROBRK_STANDSTILL_STANDSTILL'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 20, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Aducontrolbrake0x110110::set_p_adu_controbrk_standstill(uint8_t* data,
    Adu_controlbrake_0x110_110::Adu_controbrk_standstillType adu_controbrk_standstill) {
  int x = adu_controbrk_standstill;

  Byte to_set(data + 2);
  to_set.set_value(x, 4, 1);
}


Aducontrolbrake0x110110* Aducontrolbrake0x110110::set_adu_controbrk_enable(
    Adu_controlbrake_0x110_110::Adu_controbrk_enableType adu_controbrk_enable) {
  adu_controbrk_enable_ = adu_controbrk_enable;
  return this;
 }

// config detail: {'name': 'ADU_ControBrk_Enable', 'enum': {0: 'ADU_CONTROBRK_ENABLE_DISABLE', 1: 'ADU_CONTROBRK_ENABLE_ENABLE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 21, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Aducontrolbrake0x110110::set_p_adu_controbrk_enable(uint8_t* data,
    Adu_controlbrake_0x110_110::Adu_controbrk_enableType adu_controbrk_enable) {
  int x = adu_controbrk_enable;

  Byte to_set(data + 2);
  to_set.set_value(x, 5, 1);
}


Aducontrolbrake0x110110* Aducontrolbrake0x110110::set_adu_mastercylinderpressreq(
    int adu_mastercylinderpressreq) {
  adu_mastercylinderpressreq_ = adu_mastercylinderpressreq;
  return this;
 }

// config detail: {'name': 'ADU_MasterCylinderPressReq', 'offset': 0.0, 'precision': 1.0, 'len': 15, 'is_signed_var': False, 'physical_range': '[0|32000]', 'bit': 7, 'type': 'int', 'order': 'motorola', 'physical_unit': 'kpa'}
void Aducontrolbrake0x110110::set_p_adu_mastercylinderpressreq(uint8_t* data,
    int adu_mastercylinderpressreq) {
  adu_mastercylinderpressreq = ProtocolData::BoundedValue(0, 32000, adu_mastercylinderpressreq);
  int x = adu_mastercylinderpressreq;
  uint8_t t = 0;

  t = x & 0x7F;
  Byte to_set0(data + 1);
  to_set0.set_value(t, 1, 7);
  x >>= 7;

  t = x & 0xFF;
  Byte to_set1(data + 0);
  to_set1.set_value(t, 0, 8);
}

}  // namespace teshun
}  // namespace canbus
}  // namespace jmc_auto
