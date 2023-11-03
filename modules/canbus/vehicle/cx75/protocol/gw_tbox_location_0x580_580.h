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

#ifndef MODULES_CANBUS_VEHICLE_cx75_PROTOCOL_GW_TBOX_LOCATION_0X580_580_H_
#define MODULES_CANBUS_VEHICLE_cx75_PROTOCOL_GW_TBOX_LOCATION_0X580_580_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

class Gwtboxlocation0x580580 : public ::jmc_auto::drivers::canbus::ProtocolData<
                    ::jmc_auto::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Gwtboxlocation0x580580();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'name': 'TBOX_Longitude', 'enum': {268435455: 'TBOX_LONGITUDE_INVALID'}, 'precision': 1e-06, 'len': 28, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|180]', 'bit': 39, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  double tbox_longitude(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'TBOX_Latidude', 'enum': {268435455: 'TBOX_LATIDUDE_INVALID'}, 'precision': 1e-06, 'len': 28, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|90]', 'bit': 4, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  double tbox_latidude(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'TBOX_GPS_UB', 'enum': {0: 'TBOX_GPS_UB_NOT_UPDATE_GPS', 1: 'TBOX_GPS_UB_UPDATE_GPS'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 59, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_tbox_location_0x580_580::Tbox_gps_ubType tbox_gps_ub(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'TBOX_LocationSts', 'enum': {0: 'TBOX_LOCATIONSTS_NORTH_LATITUDE_EAST_LONGITUDE', 1: 'TBOX_LOCATIONSTS_NORTH_LATITUDE_WEST_LONGITUDE', 2: 'TBOX_LOCATIONSTS_SOUTH_LATITUDE_EAST_LONGITUDE', 3: 'TBOX_LOCATIONSTS_SOUTH_LATITUDE_WEST_LONGITUDE', 7: 'TBOX_LOCATIONSTS_INVALID'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|6]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_tbox_location_0x580_580::Tbox_locationstsType tbox_locationsts(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_cx75_PROTOCOL_GW_TBOX_LOCATION_0X580_580_H_
