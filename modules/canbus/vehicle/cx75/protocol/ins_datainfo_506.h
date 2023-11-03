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

#ifndef MODULES_CANBUS_VEHICLE_CX75_PROTOCOL_INS_DATAINFO_506_H_
#define MODULES_CANBUS_VEHICLE_CX75_PROTOCOL_INS_DATAINFO_506_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

class Insdatainfo506 : public ::jmc_auto::drivers::canbus::ProtocolData<
                    ::jmc_auto::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Insdatainfo506();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'name': 'INS_GpsFlag_Pos', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 7, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int ins_gpsflag_pos(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'INS_NumSV', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 15, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int ins_numsv(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'INS_GpsFlag_Heading', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 23, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int ins_gpsflag_heading(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'INS_Gps_Age', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 31, 'type': 'int', 'order': 'motorola', 'physical_unit': 's'}
  int ins_gps_age(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'INS_Car_Status', 'enum': {0: 'INS_CAR_STATUS_RECEIVEWHEELSPEED', 1: 'INS_CAR_STATUS_NORECEIVE'}, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|255]', 'bit': 39, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  int ins_car_status(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'INS_Status', 'enum': {0: 'INS_STATUS_NONE', 1: 'INS_STATUS_ATTITUDEINITIALIZATION', 2: 'INS_STATUS_NAVIGATION'}, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|255]', 'bit': 63, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  int ins_status(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_CX75_PROTOCOL_INS_DATAINFO_506_H_
