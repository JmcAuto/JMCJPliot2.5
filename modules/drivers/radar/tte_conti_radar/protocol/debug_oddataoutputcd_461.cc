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

#include "modules/drivers/radar/tte_conti_radar/protocol/debug_oddataoutputcd_461.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace jmc_auto {
namespace drivers {
namespace tte_conti_radar {

using ::jmc_auto::drivers::canbus::Byte;

Debugoddataoutputcd461::Debugoddataoutputcd461() {}
const int32_t Debugoddataoutputcd461::ID = 0x461;

void Debugoddataoutputcd461::Parse(const std::uint8_t* bytes, int32_t length,
                       TteContiRadar* ttecontiradar) const {
  ttecontiradar->mutable_debug_oddataoutputcd_461()->set_od_datacoutput_confidencelvl(od_datacoutput_confidencelvl(bytes, length));
  ttecontiradar->mutable_debug_oddataoutputcd_461()->set_od_datacoutput_type(od_datacoutput_type(bytes, length));
  ttecontiradar->mutable_debug_oddataoutputcd_461()->set_od_datacoutput_x(od_datacoutput_x(bytes, length));
  ttecontiradar->mutable_debug_oddataoutputcd_461()->set_od_datacoutput_y(od_datacoutput_y(bytes, length));
  ttecontiradar->mutable_debug_oddataoutputcd_461()->set_od_datadoutput_confidencelvl(od_datadoutput_confidencelvl(bytes, length));
  ttecontiradar->mutable_debug_oddataoutputcd_461()->set_od_datadoutput_type(od_datadoutput_type(bytes, length));
  ttecontiradar->mutable_debug_oddataoutputcd_461()->set_od_datadoutput_x(od_datadoutput_x(bytes, length));
  ttecontiradar->mutable_debug_oddataoutputcd_461()->set_od_datadoutput_y(od_datadoutput_y(bytes, length));
}

// config detail: {'name': 'od_datacoutput_confidencelvl', 'enum': {0: 'OD_DATACOUTPUT_CONFIDENCELVL_LOW', 1: 'OD_DATACOUTPUT_CONFIDENCELVL_GENERAL', 2: 'OD_DATACOUTPUT_CONFIDENCELVL_BELIEVABLE', 3: 'OD_DATACOUTPUT_CONFIDENCELVL_VERYBELIEVABLE', 4: 'OD_DATACOUTPUT_CONFIDENCELVL_RESERVED'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 2, 'type': 'enum', 'order': 'motorola', 'physical_unit': '0'}
Debug_oddataoutputcd_461::Od_datacoutput_confidencelvlType Debugoddataoutputcd461::od_datacoutput_confidencelvl(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 3);

  Debug_oddataoutputcd_461::Od_datacoutput_confidencelvlType ret =  static_cast<Debug_oddataoutputcd_461::Od_datacoutput_confidencelvlType>(x);
  return ret;
}

// config detail: {'name': 'od_datacoutput_type', 'enum': {0: 'OD_DATACOUTPUT_TYPE_UNKNOWN', 1: 'OD_DATACOUTPUT_TYPE_PEDESTRIAN', 2: 'OD_DATACOUTPUT_TYPE_FOURWHEELEDVEHICLE', 3: 'OD_DATACOUTPUT_TYPE_TWOWHEELEDVEHICLE', 4: 'OD_DATACOUTPUT_TYPE_RIDER', 5: 'OD_DATACOUTPUT_TYPE_GROUNDLOCK', 6: 'OD_DATACOUTPUT_TYPE_CONE', 7: 'OD_DATACOUTPUT_TYPE_WHEELBLOCK', 8: 'OD_DATACOUTPUT_TYPE_WATERHORSE', 9: 'OD_DATACOUTPUT_TYPE_RESERVED'}, 'precision': 1.0, 'len': 5, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|31]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': 'cm'}
Debug_oddataoutputcd_461::Od_datacoutput_typeType Debugoddataoutputcd461::od_datacoutput_type(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(3, 5);

  Debug_oddataoutputcd_461::Od_datacoutput_typeType ret =  static_cast<Debug_oddataoutputcd_461::Od_datacoutput_typeType>(x);
  return ret;
}

// config detail: {'name': 'od_datacoutput_x', 'offset': -2048.0, 'precision': 1.0, 'len': 12, 'is_signed_var': False, 'physical_range': '[-2048|2047]', 'bit': 15, 'type': 'int', 'order': 'motorola', 'physical_unit': 'cm'}
int Debugoddataoutputcd461::od_datacoutput_x(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(4, 4);
  x <<= 4;
  x |= t;

  int ret = x + -2048.000000;
  return ret;
}

// config detail: {'name': 'od_datacoutput_y', 'offset': -2048.0, 'precision': 1.0, 'len': 12, 'is_signed_var': False, 'physical_range': '[-2048|2047]', 'bit': 19, 'type': 'int', 'order': 'motorola', 'physical_unit': 'cm'}
int Debugoddataoutputcd461::od_datacoutput_y(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 4);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x + -2048.000000;
  return ret;
}

// config detail: {'name': 'od_datadoutput_confidencelvl', 'enum': {0: 'OD_DATADOUTPUT_CONFIDENCELVL_LOW', 1: 'OD_DATADOUTPUT_CONFIDENCELVL_GENERAL', 2: 'OD_DATADOUTPUT_CONFIDENCELVL_BELIEVABLE', 3: 'OD_DATADOUTPUT_CONFIDENCELVL_VERYBELIEVABLE', 4: 'OD_DATADOUTPUT_CONFIDENCELVL_RESERVED'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 34, 'type': 'enum', 'order': 'motorola', 'physical_unit': '0'}
Debug_oddataoutputcd_461::Od_datadoutput_confidencelvlType Debugoddataoutputcd461::od_datadoutput_confidencelvl(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 3);

  Debug_oddataoutputcd_461::Od_datadoutput_confidencelvlType ret =  static_cast<Debug_oddataoutputcd_461::Od_datadoutput_confidencelvlType>(x);
  return ret;
}

// config detail: {'name': 'od_datadoutput_type', 'enum': {0: 'OD_DATADOUTPUT_TYPE_UNKNOWN', 1: 'OD_DATADOUTPUT_TYPE_PEDESTRIAN', 2: 'OD_DATADOUTPUT_TYPE_FOURWHEELEDVEHICLE', 3: 'OD_DATADOUTPUT_TYPE_TWOWHEELEDVEHICLE', 4: 'OD_DATADOUTPUT_TYPE_RIDER', 5: 'OD_DATADOUTPUT_TYPE_GROUNDLOCK', 6: 'OD_DATADOUTPUT_TYPE_CONE', 7: 'OD_DATADOUTPUT_TYPE_WHEELBLOCK', 8: 'OD_DATADOUTPUT_TYPE_WATERHORSE', 9: 'OD_DATADOUTPUT_TYPE_RESERVED'}, 'precision': 1.0, 'len': 5, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|31]', 'bit': 39, 'type': 'enum', 'order': 'motorola', 'physical_unit': 'cm'}
Debug_oddataoutputcd_461::Od_datadoutput_typeType Debugoddataoutputcd461::od_datadoutput_type(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(3, 5);

  Debug_oddataoutputcd_461::Od_datadoutput_typeType ret =  static_cast<Debug_oddataoutputcd_461::Od_datadoutput_typeType>(x);
  return ret;
}

// config detail: {'name': 'od_datadoutput_x', 'offset': -2048.0, 'precision': 1.0, 'len': 12, 'is_signed_var': False, 'physical_range': '[-2048|2047]', 'bit': 47, 'type': 'int', 'order': 'motorola', 'physical_unit': 'cm'}
int Debugoddataoutputcd461::od_datadoutput_x(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 6);
  int32_t t = t1.get_byte(4, 4);
  x <<= 4;
  x |= t;

  int ret = x + -2048.000000;
  return ret;
}

// config detail: {'name': 'od_datadoutput_y', 'offset': -2048.0, 'precision': 1.0, 'len': 12, 'is_signed_var': False, 'physical_range': '[-2048|2047]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': 'cm'}
int Debugoddataoutputcd461::od_datadoutput_y(const std::uint8_t* bytes, int32_t length) const {
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
