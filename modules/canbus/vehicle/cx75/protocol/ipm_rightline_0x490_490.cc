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

#include "modules/canbus/vehicle/cx75/protocol/ipm_rightline_0x490_490.h"

#include "modules/drivers/canbus/common/byte.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

using ::jmc_auto::drivers::canbus::Byte;

const int32_t Ipmrightline0x490490::ID = 0x490;

// public
Ipmrightline0x490490::Ipmrightline0x490490() { Reset(); }

uint32_t Ipmrightline0x490490::GetPeriod() const {
  // TODO modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Ipmrightline0x490490::UpdateData(uint8_t* data) {
  set_p_ipm_rightline_dy(data, ipm_rightline_dy_);
  set_p_ipm_rightline_dx_lookhead(data, ipm_rightline_dx_lookhead_);
  set_p_ipm_rightline_hor_curve(data, ipm_rightline_hor_curve_);
  set_p_ipm_rightline_yawangle(data, ipm_rightline_yawangle_);
  set_p_ipm_rightline_dx_start(data, ipm_rightline_dx_start_);
}

void Ipmrightline0x490490::Reset() {
  // TODO you should check this manually
  ipm_rightline_dy_ = 0.0;
  ipm_rightline_dx_lookhead_ = 0.0;
  ipm_rightline_hor_curve_ = 0.0;
  ipm_rightline_yawangle_ = 0.0;
  ipm_rightline_dx_start_ = 0.0;
}

Ipmrightline0x490490* Ipmrightline0x490490::set_ipm_rightline_dy(
    double ipm_rightline_dy) {
  ipm_rightline_dy_ = ipm_rightline_dy;
  return this;
 }

// config detail: {'name': 'IPM_RightLine_dy', 'enum': {0: 'IPM_RIGHTLINE_DY_INITIAL_VALUE_NO_LINE'}, 'precision': 0.00390625, 'len': 12, 'is_signed_var': False, 'offset': -8.0, 'physical_range': '[-8|7.99609375]', 'bit': 15, 'type': 'enum', 'order': 'motorola', 'physical_unit': 'm'}
void Ipmrightline0x490490::set_p_ipm_rightline_dy(uint8_t* data,
    double ipm_rightline_dy) {
  int x = (ipm_rightline_dy - -8.000000) / 0.003906;
  uint8_t t = 0;

  t = x & 0xF;
  Byte to_set0(data + 2);
  to_set0.set_value(t, 4, 4);
  x >>= 4;

  t = x & 0xFF;
  Byte to_set1(data + 1);
  to_set1.set_value(t, 0, 8);
}


Ipmrightline0x490490* Ipmrightline0x490490::set_ipm_rightline_dx_lookhead(
    double ipm_rightline_dx_lookhead) {
  ipm_rightline_dx_lookhead_ = ipm_rightline_dx_lookhead;
  return this;
 }

// config detail: {'name': 'IPM_RightLine_dx_lookhead', 'offset': 0.0, 'precision': 0.25, 'len': 9, 'is_signed_var': False, 'physical_range': '[0|127.75]', 'bit': 19, 'type': 'double', 'order': 'motorola', 'physical_unit': 'm'}
void Ipmrightline0x490490::set_p_ipm_rightline_dx_lookhead(uint8_t* data,
    double ipm_rightline_dx_lookhead) {
  ipm_rightline_dx_lookhead = ProtocolData::BoundedValue(0.0, 127.75, ipm_rightline_dx_lookhead);
  int x = ipm_rightline_dx_lookhead / 0.250000;
  uint8_t t = 0;

  t = x & 0x1F;
  Byte to_set0(data + 3);
  to_set0.set_value(t, 3, 5);
  x >>= 5;

  t = x & 0xF;
  Byte to_set1(data + 2);
  to_set1.set_value(t, 0, 4);
}


Ipmrightline0x490490* Ipmrightline0x490490::set_ipm_rightline_hor_curve(
    double ipm_rightline_hor_curve) {
  ipm_rightline_hor_curve_ = ipm_rightline_hor_curve;
  return this;
 }

// config detail: {'name': 'IPM_RightLine_hor_curve', 'enum': {0: 'IPM_RIGHTLINE_HOR_CURVE_INITIAL_VALUE_NO_CURVATURE'}, 'precision': 0.0001, 'len': 10, 'is_signed_var': False, 'offset': -0.015, 'physical_range': '[-0.015|0.0513]', 'bit': 26, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Ipmrightline0x490490::set_p_ipm_rightline_hor_curve(uint8_t* data,
    double ipm_rightline_hor_curve) {
  int x = (ipm_rightline_hor_curve - -0.015000) / 0.000100;
  uint8_t t = 0;

  t = x & 0x7F;
  Byte to_set0(data + 4);
  to_set0.set_value(t, 1, 7);
  x >>= 7;

  t = x & 0x7;
  Byte to_set1(data + 3);
  to_set1.set_value(t, 0, 3);
}


Ipmrightline0x490490* Ipmrightline0x490490::set_ipm_rightline_yawangle(
    double ipm_rightline_yawangle) {
  ipm_rightline_yawangle_ = ipm_rightline_yawangle;
  return this;
 }

// config detail: {'name': 'IPM_RightLine_yawangle', 'offset': -1.0, 'precision': 0.000488281, 'len': 12, 'is_signed_var': False, 'physical_range': '[-1|0.999510695]', 'bit': 47, 'type': 'double', 'order': 'motorola', 'physical_unit': ''}
void Ipmrightline0x490490::set_p_ipm_rightline_yawangle(uint8_t* data,
    double ipm_rightline_yawangle) {
  ipm_rightline_yawangle = ProtocolData::BoundedValue(-1.0, 0.999510695, ipm_rightline_yawangle);
  int x = (ipm_rightline_yawangle - -1.000000) / 0.000488;
  uint8_t t = 0;

  t = x & 0xF;
  Byte to_set0(data + 6);
  to_set0.set_value(t, 4, 4);
  x >>= 4;

  t = x & 0xFF;
  Byte to_set1(data + 5);
  to_set1.set_value(t, 0, 8);
}


Ipmrightline0x490490* Ipmrightline0x490490::set_ipm_rightline_dx_start(
    double ipm_rightline_dx_start) {
  ipm_rightline_dx_start_ = ipm_rightline_dx_start;
  return this;
 }

// config detail: {'name': 'IPM_RightLine_dx_start', 'offset': 0.0, 'precision': 0.25, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|63.75]', 'bit': 7, 'type': 'double', 'order': 'motorola', 'physical_unit': 'm'}
void Ipmrightline0x490490::set_p_ipm_rightline_dx_start(uint8_t* data,
    double ipm_rightline_dx_start) {
  ipm_rightline_dx_start = ProtocolData::BoundedValue(0.0, 63.75, ipm_rightline_dx_start);
  int x = ipm_rightline_dx_start / 0.250000;

  Byte to_set(data + 0);
  to_set.set_value(x, 0, 8);
}

}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto
