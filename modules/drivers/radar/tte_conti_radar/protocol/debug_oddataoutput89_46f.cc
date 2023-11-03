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

#include "modules/drivers/radar/tte_conti_radar/protocol/debug_oddataoutput89_46f.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace jmc_auto {
namespace drivers {
namespace tte_conti_radar {

using ::jmc_auto::drivers::canbus::Byte;

Debugoddataoutput8946f::Debugoddataoutput8946f() {}
const int32_t Debugoddataoutput8946f::ID = 0x46F;

void Debugoddataoutput8946f::Parse(const std::uint8_t* bytes, int32_t length,
                       TteContiRadar* ttecontiradar) const {
  ttecontiradar->mutable_debug_oddataoutput89_46f()->set_od_data8output_confidencelvl(od_data8output_confidencelvl(bytes, length));
  ttecontiradar->mutable_debug_oddataoutput89_46f()->set_od_data8output_type(od_data8output_type(bytes, length));
  ttecontiradar->mutable_debug_oddataoutput89_46f()->set_od_data8output_x(od_data8output_x(bytes, length));
  ttecontiradar->mutable_debug_oddataoutput89_46f()->set_od_data8output_y(od_data8output_y(bytes, length));
  ttecontiradar->mutable_debug_oddataoutput89_46f()->set_od_data9output_confidencelvl(od_data9output_confidencelvl(bytes, length));
  ttecontiradar->mutable_debug_oddataoutput89_46f()->set_od_data9output_type(od_data9output_type(bytes, length));
  ttecontiradar->mutable_debug_oddataoutput89_46f()->set_od_data9output_x(od_data9output_x(bytes, length));
  ttecontiradar->mutable_debug_oddataoutput89_46f()->set_od_data9output_y(od_data9output_y(bytes, length));
}

// config detail: {'name': 'od_data8output_confidencelvl', 'enum': {0: 'OD_DATA8OUTPUT_CONFIDENCELVL_LOW', 1: 'OD_DATA8OUTPUT_CONFIDENCELVL_GENERAL', 2: 'OD_DATA8OUTPUT_CONFIDENCELVL_BELIEVABLE', 3: 'OD_DATA8OUTPUT_CONFIDENCELVL_VERYBELIEVABLE', 4: 'OD_DATA8OUTPUT_CONFIDENCELVL_RESERVED'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 2, 'type': 'enum', 'order': 'motorola', 'physical_unit': '0'}
Debug_oddataoutput89_46f::Od_data8output_confidencelvlType Debugoddataoutput8946f::od_data8output_confidencelvl(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 3);

  Debug_oddataoutput89_46f::Od_data8output_confidencelvlType ret =  static_cast<Debug_oddataoutput89_46f::Od_data8output_confidencelvlType>(x);
  return ret;
}

// config detail: {'name': 'od_data8output_type', 'enum': {0: 'OD_DATA8OUTPUT_TYPE_UNKNOWN', 1: 'OD_DATA8OUTPUT_TYPE_PEDESTRIAN', 2: 'OD_DATA8OUTPUT_TYPE_FOURWHEELEDVEHICLE', 3: 'OD_DATA8OUTPUT_TYPE_TWOWHEELEDVEHICLE', 4: 'OD_DATA8OUTPUT_TYPE_RIDER', 5: 'OD_DATA8OUTPUT_TYPE_GROUNDLOCK', 6: 'OD_DATA8OUTPUT_TYPE_CONE', 7: 'OD_DATA8OUTPUT_TYPE_WHEELBLOCK', 8: 'OD_DATA8OUTPUT_TYPE_WATERHORSE', 9: 'OD_DATA8OUTPUT_TYPE_RESERVED'}, 'precision': 1.0, 'len': 5, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|31]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': 'cm'}
Debug_oddataoutput89_46f::Od_data8output_typeType Debugoddataoutput8946f::od_data8output_type(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(3, 5);

  Debug_oddataoutput89_46f::Od_data8output_typeType ret =  static_cast<Debug_oddataoutput89_46f::Od_data8output_typeType>(x);
  return ret;
}

// config detail: {'name': 'od_data8output_x', 'offset': -2048.0, 'precision': 1.0, 'len': 12, 'is_signed_var': False, 'physical_range': '[-2048|2047]', 'bit': 15, 'type': 'int', 'order': 'motorola', 'physical_unit': 'cm'}
int Debugoddataoutput8946f::od_data8output_x(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(4, 4);
  x <<= 4;
  x |= t;

  int ret = x + -2048.000000;
  return ret;
}

// config detail: {'name': 'od_data8output_y', 'offset': -2048.0, 'precision': 1.0, 'len': 12, 'is_signed_var': False, 'physical_range': '[-2048|2047]', 'bit': 19, 'type': 'int', 'order': 'motorola', 'physical_unit': 'cm'}
int Debugoddataoutput8946f::od_data8output_y(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 4);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x + -2048.000000;
  return ret;
}

// config detail: {'name': 'od_data9output_confidencelvl', 'enum': {0: 'OD_DATA9OUTPUT_CONFIDENCELVL_LOW', 1: 'OD_DATA9OUTPUT_CONFIDENCELVL_GENERAL', 2: 'OD_DATA9OUTPUT_CONFIDENCELVL_BELIEVABLE', 3: 'OD_DATA9OUTPUT_CONFIDENCELVL_VERYBELIEVABLE', 4: 'OD_DATA9OUTPUT_CONFIDENCELVL_RESERVED'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 34, 'type': 'enum', 'order': 'motorola', 'physical_unit': '0'}
Debug_oddataoutput89_46f::Od_data9output_confidencelvlType Debugoddataoutput8946f::od_data9output_confidencelvl(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 3);

  Debug_oddataoutput89_46f::Od_data9output_confidencelvlType ret =  static_cast<Debug_oddataoutput89_46f::Od_data9output_confidencelvlType>(x);
  return ret;
}

// config detail: {'name': 'od_data9output_type', 'enum': {0: 'OD_DATA9OUTPUT_TYPE_UNKNOWN', 1: 'OD_DATA9OUTPUT_TYPE_PEDESTRIAN', 2: 'OD_DATA9OUTPUT_TYPE_FOURWHEELEDVEHICLE', 3: 'OD_DATA9OUTPUT_TYPE_TWOWHEELEDVEHICLE', 4: 'OD_DATA9OUTPUT_TYPE_RIDER', 5: 'OD_DATA9OUTPUT_TYPE_GROUNDLOCK', 6: 'OD_DATA9OUTPUT_TYPE_CONE', 7: 'OD_DATA9OUTPUT_TYPE_WHEELBLOCK', 8: 'OD_DATA9OUTPUT_TYPE_WATERHORSE', 9: 'OD_DATA9OUTPUT_TYPE_RESERVED'}, 'precision': 1.0, 'len': 5, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|31]', 'bit': 39, 'type': 'enum', 'order': 'motorola', 'physical_unit': 'cm'}
Debug_oddataoutput89_46f::Od_data9output_typeType Debugoddataoutput8946f::od_data9output_type(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(3, 5);

  Debug_oddataoutput89_46f::Od_data9output_typeType ret =  static_cast<Debug_oddataoutput89_46f::Od_data9output_typeType>(x);
  return ret;
}

// config detail: {'name': 'od_data9output_x', 'offset': -2048.0, 'precision': 1.0, 'len': 12, 'is_signed_var': False, 'physical_range': '[-2048|2047]', 'bit': 47, 'type': 'int', 'order': 'motorola', 'physical_unit': 'cm'}
int Debugoddataoutput8946f::od_data9output_x(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 6);
  int32_t t = t1.get_byte(4, 4);
  x <<= 4;
  x |= t;

  int ret = x + -2048.000000;
  return ret;
}

// config detail: {'name': 'od_data9output_y', 'offset': -2048.0, 'precision': 1.0, 'len': 12, 'is_signed_var': False, 'physical_range': '[-2048|2047]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': 'cm'}
int Debugoddataoutput8946f::od_data9output_y(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 4);

  Byte t1(bytes + 7);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x + -2048.000000;
  return ret;
}
}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto
