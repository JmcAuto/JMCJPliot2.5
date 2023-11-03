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

#include "modules/canbus/vehicle/cx75/protocol/ins_datainfo_506.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

using ::jmc_auto::drivers::canbus::Byte;

Insdatainfo506::Insdatainfo506() {}
const int32_t Insdatainfo506::ID = 0x506;

void Insdatainfo506::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_cx75()->mutable_ins_datainfo_506()->set_ins_gpsflag_pos(ins_gpsflag_pos(bytes, length));
  chassis->mutable_cx75()->mutable_ins_datainfo_506()->set_ins_numsv(ins_numsv(bytes, length));
  chassis->mutable_cx75()->mutable_ins_datainfo_506()->set_ins_gpsflag_heading(ins_gpsflag_heading(bytes, length));
  chassis->mutable_cx75()->mutable_ins_datainfo_506()->set_ins_gps_age(ins_gps_age(bytes, length));
  chassis->mutable_cx75()->mutable_ins_datainfo_506()->set_ins_car_status(ins_car_status(bytes, length));
  chassis->mutable_cx75()->mutable_ins_datainfo_506()->set_ins_status(ins_status(bytes, length));
}

// config detail: {'name': 'ins_gpsflag_pos', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 7, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Insdatainfo506::ins_gpsflag_pos(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  uint32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'ins_numsv', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 15, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Insdatainfo506::ins_numsv(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  uint32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'ins_gpsflag_heading', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 23, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Insdatainfo506::ins_gpsflag_heading(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  uint32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'ins_gps_age', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 31, 'type': 'int', 'order': 'motorola', 'physical_unit': 's'}
int Insdatainfo506::ins_gps_age(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  uint32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'ins_car_status', 'enum': {0: 'INS_CAR_STATUS_RECEIVEWHEELSPEED', 1: 'INS_CAR_STATUS_NORECEIVE'}, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|255]', 'bit': 39, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
int Insdatainfo506::ins_car_status(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  uint32_t x = t0.get_byte(0, 8);

  //Ins_datainfo_506::Ins_car_statusType ret =  static_cast<Ins_datainfo_506::Ins_car_statusType>(x);
  int ret = x;
  return ret;
}

// config detail: {'name': 'ins_status', 'enum': {0: 'INS_STATUS_NONE', 1: 'INS_STATUS_ATTITUDEINITIALIZATION', 2: 'INS_STATUS_NAVIGATION'}, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|255]', 'bit': 63, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
int Insdatainfo506::ins_status(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  uint32_t x = t0.get_byte(0, 8);

  //Ins_datainfo_506::Ins_statusType ret =  static_cast<Ins_datainfo_506::Ins_statusType>(x);
  int ret = x;
  return ret;
}
}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto
