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

#include "modules/canbus/vehicle/cx75/protocol/mrr_0x238_238.h"

#include "modules/drivers/canbus/common/byte.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

using ::jmc_auto::drivers::canbus::Byte;

const int32_t Mrr0x238238::ID = 0x238;

// public
Mrr0x238238::Mrr0x238238() { Reset(); }

uint32_t Mrr0x238238::GetPeriod() const {
  // TODO modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Mrr0x238238::UpdateData(uint8_t* data) {
  set_p_acc_tgtaxlowercomftband(data, acc_tgtaxlowercomftband_);
  set_p_acc_tgtaxupperlim(data, acc_tgtaxupperlim_);
  set_p_acc_tgtaxlowerlim(data, acc_tgtaxlowerlim_);
  set_p_acc_tgtax(data, acc_tgtax_);
  set_p_rolling_counter_0x238(data, rolling_counter_0x238_);
  set_p_acc_tgtaxuppercomftband(data, acc_tgtaxuppercomftband_);
  set_p_checksum_0x238(data, checksum_0x238_);
}

void Mrr0x238238::Reset() {
  // TODO you should check this manually
  acc_tgtaxlowercomftband_ = 0.0;
  acc_tgtaxupperlim_ = 0.0;
  acc_tgtaxlowerlim_ = 0.0;
  acc_tgtax_ = 0.0;
  rolling_counter_0x238_ = 0;
  checksum_0x238_ = 0;
  acc_tgtaxuppercomftband_ = 0.0;
}

void Mrr0x238238::checksum_rolling() {
  // TODO you should check this manually
  //rolling_counter_0x238_++;
  //rolling_counter_0x238_=rolling_counter_0x238_<=15?rolling_counter_0x238_:0;
  //rolling_counter_0x238_ = ProtocolData::BoundedValue(0, 15, rolling_counter_0x238_);
}

Mrr0x238238* Mrr0x238238::set_acc_tgtaxlowercomftband(
    double acc_tgtaxlowercomftband) {
  acc_tgtaxlowercomftband_ = acc_tgtaxlowercomftband;
  return this;
 }

// config detail: {'name': 'ACC_TgtAxLowerComftBand', 'offset': 0.0, 'precision': 0.05, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|3.2]', 'bit': 15, 'type': 'double', 'order': 'motorola', 'physical_unit': 'm s2'}
void Mrr0x238238::set_p_acc_tgtaxlowercomftband(uint8_t* data,
    double acc_tgtaxlowercomftband) {
  acc_tgtaxlowercomftband = ProtocolData::BoundedValue(0.0, 3.2, acc_tgtaxlowercomftband);
  int x = acc_tgtaxlowercomftband / 0.050000;

  Byte to_set(data + 1);
  to_set.set_value(x, 0, 8);
}


Mrr0x238238* Mrr0x238238::set_acc_tgtaxupperlim(
    double acc_tgtaxupperlim) {
  acc_tgtaxupperlim_ = acc_tgtaxupperlim;
  return this;
 }

// config detail: {'name': 'ACC_TgtAxUpperLim', 'offset': 0.0, 'precision': 0.1, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|12.7]', 'bit': 23, 'type': 'double', 'order': 'motorola', 'physical_unit': 'm s3'}
void Mrr0x238238::set_p_acc_tgtaxupperlim(uint8_t* data,
    double acc_tgtaxupperlim) {
  acc_tgtaxupperlim = ProtocolData::BoundedValue(0.0, 12.7, acc_tgtaxupperlim);
  int x = acc_tgtaxupperlim / 0.100000;

  Byte to_set(data + 2);
  to_set.set_value(x, 0, 8);
}


Mrr0x238238* Mrr0x238238::set_acc_tgtaxlowerlim(
    double acc_tgtaxlowerlim) {
  acc_tgtaxlowerlim_ = acc_tgtaxlowerlim;
  return this;
 }

// config detail: {'name': 'ACC_TgtAxLowerLim', 'offset': -12.7, 'precision': 0.1, 'len': 8, 'is_signed_var': False, 'physical_range': '[-12.7|0]', 'bit': 31, 'type': 'double', 'order': 'motorola', 'physical_unit': 'm s3'}
void Mrr0x238238::set_p_acc_tgtaxlowerlim(uint8_t* data,
    double acc_tgtaxlowerlim) {
  acc_tgtaxlowerlim = ProtocolData::BoundedValue(-12.7, 0.0, acc_tgtaxlowerlim);
  int x = (acc_tgtaxlowerlim - -12.700000) / 0.100000;

  Byte to_set(data + 3);
  to_set.set_value(x, 0, 8);
}


Mrr0x238238* Mrr0x238238::set_acc_tgtax(
    double acc_tgtax) {
  acc_tgtax_ = acc_tgtax;
  return this;
 }

// config detail: {'name': 'ACC_TgtAx', 'offset': -5.0, 'precision': 0.05, 'len': 11, 'is_signed_var': False, 'physical_range': '[-5|7.75]', 'bit': 47, 'type': 'double', 'order': 'motorola', 'physical_unit': 'm s2'}
void Mrr0x238238::set_p_acc_tgtax(uint8_t* data,
    double acc_tgtax) {
  acc_tgtax = ProtocolData::BoundedValue(-5.0, 7.75, acc_tgtax);
  int x = (acc_tgtax - -5.000000) / 0.050000;
  uint8_t t = 0;

  t = x & 0x7;
  Byte to_set0(data + 6);
  to_set0.set_value(t, 5, 3);
  x >>= 3;

  t = x & 0xFF;
  Byte to_set1(data + 5);
  to_set1.set_value(t, 0, 8);
}


Mrr0x238238* Mrr0x238238::set_rolling_counter_0x238(
    int rolling_counter_0x238) {
  rolling_counter_0x238_ = rolling_counter_0x238;
  return this;
 }

// config detail: {'name': 'Rolling_counter_0x238', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
void Mrr0x238238::set_p_rolling_counter_0x238(uint8_t* data,
    int rolling_counter_0x238) {
  //rolling_counter_0x238_++;
  //rolling_counter_0x238_=rolling_counter_0x238_<=15?rolling_counter_0x238_:0;
  //rolling_counter_0x238_ = ProtocolData::BoundedValue(0, 15, rolling_counter_0x238_);
  //int x = rolling_counter_0x238_;
  //Byte to_set(data + 6);
  //to_set.set_value(x, 0, 4);
}


Mrr0x238238* Mrr0x238238::set_checksum_0x238(
    int checksum_0x238) {
  checksum_0x238_ = checksum_0x238;
  return this;
 }

// config detail: {'name': 'Checksum_0x238', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
void Mrr0x238238::set_p_checksum_0x238(uint8_t* data,
    int checksum_0x238) {
  //checksum_0x238 = (data[0]^data[1]^data[2]^data[3]^data[4]^data[5]^data[6]);
  //checksum_0x238 = ProtocolData::BoundedValue(0, 255, checksum_0x238);
  //int x = checksum_0x238;

  //Byte to_set(data + 7);
  //to_set.set_value(x, 0, 8);
}


Mrr0x238238* Mrr0x238238::set_acc_tgtaxuppercomftband(
    double acc_tgtaxuppercomftband) {
  acc_tgtaxuppercomftband_ = acc_tgtaxuppercomftband;
  return this;
 }

// config detail: {'name': 'ACC_TgtAxUpperComftBand', 'offset': 0.0, 'precision': 0.05, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|3.2]', 'bit': 7, 'type': 'double', 'order': 'motorola', 'physical_unit': 'm s2'}
void Mrr0x238238::set_p_acc_tgtaxuppercomftband(uint8_t* data,
    double acc_tgtaxuppercomftband) {
  acc_tgtaxuppercomftband = ProtocolData::BoundedValue(0.0, 3.2, acc_tgtaxuppercomftband);
  int x = acc_tgtaxuppercomftband / 0.050000;

  Byte to_set(data + 0);
  to_set.set_value(x, 0, 8);
}

}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto
