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

#include "modules/canbus/vehicle/teshun/protocol/adu_controleps2_0x100_100.h"

#include "modules/drivers/canbus/common/byte.h"

namespace jmc_auto {
namespace canbus {
namespace teshun {

using ::jmc_auto::drivers::canbus::Byte;

const int32_t Aducontroleps20x100100::ID = 0x100;

// public
Aducontroleps20x100100::Aducontroleps20x100100() { Reset(); }

uint32_t Aducontroleps20x100100::GetPeriod() const {
  // TODO modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Aducontroleps20x100100::UpdateData(uint8_t* data) {
  
  set_p_adu_100h_messagecounter(data, adu_100h_messagecounter_);
  set_p_adu_ctrsteeringwheelanglespeed(data, adu_ctrsteeringwheelanglespeed_);
  set_p_adu_controsteeringwheelangle(data, adu_controsteeringwheelangle_);
  set_p_adu_controepsenable(data, adu_controepsenable_);
set_p_adu_100h_messagechecksum(data, adu_100h_messagechecksum_);
}

void Aducontroleps20x100100::Reset() {
  // TODO you should check this manually
  adu_100h_messagechecksum_ = 0;
  adu_100h_messagecounter_ = 0;
  adu_ctrsteeringwheelanglespeed_ = 0;
  adu_controsteeringwheelangle_ = 0.0;
  adu_controepsenable_ = Adu_controleps2_0x100_100::ADU_CONTROEPSENABLE_DISABLE;
}

Aducontroleps20x100100* Aducontroleps20x100100::set_adu_100h_messagechecksum(
    int adu_100h_messagechecksum) {
  adu_100h_messagechecksum_ = adu_100h_messagechecksum;
  return this;
 }

// config detail: {'name': 'ADU_100h_MessageChecksum', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
void Aducontroleps20x100100::set_p_adu_100h_messagechecksum(uint8_t* data,
    int adu_100h_messagechecksum) {
     
  //adu_100h_messagechecksum  = (data[0]^data[1]^data[2]^data[3]^data[4]^data[5]^data[6]);
  //AINFO<<"set_p_adu_100h_messagechecksum"<<data[4];
  //adu_100h_messagechecksum = ProtocolData::BoundedValue(0, 255, adu_100h_messagechecksum);
  int x = adu_100h_messagechecksum;

  Byte to_set(data + 7);
  to_set.set_value(x, 0, 8);
}


Aducontroleps20x100100* Aducontroleps20x100100::set_adu_100h_messagecounter(
    int adu_100h_messagecounter) {
  adu_100h_messagecounter_ = adu_100h_messagecounter;
  return this;
 }

// config detail: {'name': 'ADU_100h_MessageCounter', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
void Aducontroleps20x100100::set_p_adu_100h_messagecounter(uint8_t* data,
    int adu_100h_messagecounter) {
  adu_100h_messagecounter = ProtocolData::BoundedValue(0, 15, adu_100h_messagecounter);
  int x = adu_100h_messagecounter;

  Byte to_set(data + 6);
  to_set.set_value(x, 0, 4);
}


Aducontroleps20x100100* Aducontroleps20x100100::set_adu_ctrsteeringwheelanglespeed(
    int adu_ctrsteeringwheelanglespeed) {
  adu_ctrsteeringwheelanglespeed_ = adu_ctrsteeringwheelanglespeed;
  return this;
 }

// config detail: {'name': 'ADU_CtrSteeringwheelanglespeed', 'offset': 0.0, 'precision': 1.0, 'len': 9, 'is_signed_var': False, 'physical_range': '[0|510]', 'bit': 31, 'type': 'int', 'order': 'motorola', 'physical_unit': 'deg/s'}
void Aducontroleps20x100100::set_p_adu_ctrsteeringwheelanglespeed(uint8_t* data,
    int adu_ctrsteeringwheelanglespeed) {
  adu_ctrsteeringwheelanglespeed = ProtocolData::BoundedValue(0, 510, adu_ctrsteeringwheelanglespeed);
  int x = adu_ctrsteeringwheelanglespeed;
  uint8_t t = 0;

  t = x & 0x1;
  Byte to_set0(data + 4);
  to_set0.set_value(t, 7, 1);
  x >>= 1;

  t = x & 0xFF;
  Byte to_set1(data + 3);
  to_set1.set_value(t, 0, 8);
}


Aducontroleps20x100100* Aducontroleps20x100100::set_adu_controsteeringwheelangle(
    double adu_controsteeringwheelangle) {
  adu_controsteeringwheelangle_ = adu_controsteeringwheelangle;
  return this;
 }

// config detail: {'name': 'ADU_ControSteeringwheelangle', 'offset': -612.0, 'precision': 0.1, 'len': 16, 'is_signed_var': False, 'physical_range': '[-612|612]', 'bit': 15, 'type': 'double', 'order': 'motorola', 'physical_unit': 'degree'}
void Aducontroleps20x100100::set_p_adu_controsteeringwheelangle(uint8_t* data,
    double adu_controsteeringwheelangle) {
  adu_controsteeringwheelangle = ProtocolData::BoundedValue(-612.0, 612.0, adu_controsteeringwheelangle);
  int x = (adu_controsteeringwheelangle - -612.000000) / 0.100000;
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(data + 2);
  to_set0.set_value(t, 0, 8);
  x >>= 8;

  t = x & 0xFF;
  Byte to_set1(data + 1);
  to_set1.set_value(t, 0, 8);
}


Aducontroleps20x100100* Aducontroleps20x100100::set_adu_controepsenable(
    Adu_controleps2_0x100_100::Adu_controepsenableType adu_controepsenable) {
  adu_controepsenable_ = adu_controepsenable;
  return this;
 }

// config detail: {'name': 'ADU_ControEpsEnable', 'enum': {0: 'ADU_CONTROEPSENABLE_DISABLE', 1: 'ADU_CONTROEPSENABLE_ENABLE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 0, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Aducontroleps20x100100::set_p_adu_controepsenable(uint8_t* data,
    Adu_controleps2_0x100_100::Adu_controepsenableType adu_controepsenable) {
  int x = adu_controepsenable;

  Byte to_set(data + 0);
  to_set.set_value(x, 0, 1);
}

}  // namespace teshun
}  // namespace canbus
}  // namespace jmc_auto
