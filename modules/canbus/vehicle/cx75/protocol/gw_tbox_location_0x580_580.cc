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

#include "modules/canbus/vehicle/cx75/protocol/gw_tbox_location_0x580_580.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

using ::jmc_auto::drivers::canbus::Byte;

Gwtboxlocation0x580580::Gwtboxlocation0x580580() {}
const int32_t Gwtboxlocation0x580580::ID = 0x580;

void Gwtboxlocation0x580580::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_cx75()->mutable_gw_tbox_location_0x580_580()->set_tbox_longitude(tbox_longitude(bytes, length));
  chassis->mutable_cx75()->mutable_gw_tbox_location_0x580_580()->set_tbox_latidude(tbox_latidude(bytes, length));
  chassis->mutable_cx75()->mutable_gw_tbox_location_0x580_580()->set_tbox_gps_ub(tbox_gps_ub(bytes, length));
  chassis->mutable_cx75()->mutable_gw_tbox_location_0x580_580()->set_tbox_locationsts(tbox_locationsts(bytes, length));
}

// config detail: {'name': 'tbox_longitude', 'enum': {268435455: 'TBOX_LONGITUDE_INVALID'}, 'precision': 1e-06, 'len': 28, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|180]', 'bit': 39, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
double Gwtboxlocation0x580580::tbox_longitude(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 5);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  Byte t2(bytes + 6);
  t = t2.get_byte(0, 8);
  x <<= 8;
  x |= t;

  Byte t3(bytes + 7);
  t = t3.get_byte(4, 4);
  x <<= 4;
  x |= t;

  double ret =  static_cast<double>(x);
  return ret;
}

// config detail: {'name': 'tbox_latidude', 'enum': {268435455: 'TBOX_LATIDUDE_INVALID'}, 'precision': 1e-06, 'len': 28, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|90]', 'bit': 4, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
double Gwtboxlocation0x580580::tbox_latidude(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 5);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  Byte t2(bytes + 2);
  t = t2.get_byte(0, 8);
  x <<= 8;
  x |= t;

  Byte t3(bytes + 3);
  t = t3.get_byte(1, 7);
  x <<= 7;
  x |= t;

  double ret =  static_cast<double>(x);
  return ret;
}

// config detail: {'name': 'tbox_gps_ub', 'enum': {0: 'TBOX_GPS_UB_NOT_UPDATE_GPS', 1: 'TBOX_GPS_UB_UPDATE_GPS'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 59, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_tbox_location_0x580_580::Tbox_gps_ubType Gwtboxlocation0x580580::tbox_gps_ub(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(3, 1);

  Gw_tbox_location_0x580_580::Tbox_gps_ubType ret =  static_cast<Gw_tbox_location_0x580_580::Tbox_gps_ubType>(x);
  return ret;
}

// config detail: {'name': 'tbox_locationsts', 'enum': {0: 'TBOX_LOCATIONSTS_NORTH_LATITUDE_EAST_LONGITUDE', 1: 'TBOX_LOCATIONSTS_NORTH_LATITUDE_WEST_LONGITUDE', 2: 'TBOX_LOCATIONSTS_SOUTH_LATITUDE_EAST_LONGITUDE', 3: 'TBOX_LOCATIONSTS_SOUTH_LATITUDE_WEST_LONGITUDE', 7: 'TBOX_LOCATIONSTS_INVALID'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|6]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_tbox_location_0x580_580::Tbox_locationstsType Gwtboxlocation0x580580::tbox_locationsts(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(5, 3);

  Gw_tbox_location_0x580_580::Tbox_locationstsType ret =  static_cast<Gw_tbox_location_0x580_580::Tbox_locationstsType>(x);
  return ret;
}
}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto
