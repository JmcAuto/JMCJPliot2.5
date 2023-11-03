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

#ifndef MODULES_DRIVERS_RADAR_TTE_CONTI_RADAR_PROTOCOL_DEBUG_ODDATAOUTPUT67_46E_H_
#define MODULES_DRIVERS_RADAR_TTE_CONTI_RADAR_PROTOCOL_DEBUG_ODDATAOUTPUT67_46E_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/drivers/proto/tte_conti_radar.pb.h"

namespace jmc_auto {
namespace drivers {
namespace tte_conti_radar {

class Debugoddataoutput6746e : public ::jmc_auto::drivers::canbus::ProtocolData<
                    ::jmc_auto::drivers::TteContiRadar> {
 public:
  static const int32_t ID;
  Debugoddataoutput6746e();
  void Parse(const std::uint8_t* bytes, int32_t length,
                   TteContiRadar* ttecontiradar) const override;

 private:

  // config detail: {'name': 'OD_Data7Output_ConfidenceLvl', 'enum': {0: 'OD_DATA7OUTPUT_CONFIDENCELVL_LOW', 1: 'OD_DATA7OUTPUT_CONFIDENCELVL_GENERAL', 2: 'OD_DATA7OUTPUT_CONFIDENCELVL_BELIEVABLE', 3: 'OD_DATA7OUTPUT_CONFIDENCELVL_VERYBELIEVABLE', 4: 'OD_DATA7OUTPUT_CONFIDENCELVL_RESERVED'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 34, 'type': 'enum', 'order': 'motorola', 'physical_unit': '0'}
  Debug_oddataoutput67_46e::Od_data7output_confidencelvlType od_data7output_confidencelvl(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'OD_Data7Output_Type', 'enum': {0: 'OD_DATA7OUTPUT_TYPE_UNKNOWN', 1: 'OD_DATA7OUTPUT_TYPE_PEDESTRIAN', 2: 'OD_DATA7OUTPUT_TYPE_FOURWHEELEDVEHICLE', 3: 'OD_DATA7OUTPUT_TYPE_TWOWHEELEDVEHICLE', 4: 'OD_DATA7OUTPUT_TYPE_RIDER', 5: 'OD_DATA7OUTPUT_TYPE_GROUNDLOCK', 6: 'OD_DATA7OUTPUT_TYPE_CONE', 7: 'OD_DATA7OUTPUT_TYPE_WHEELBLOCK', 8: 'OD_DATA7OUTPUT_TYPE_WATERHORSE', 9: 'OD_DATA7OUTPUT_TYPE_RESERVED'}, 'precision': 1.0, 'len': 5, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|31]', 'bit': 39, 'type': 'enum', 'order': 'motorola', 'physical_unit': 'cm'}
  Debug_oddataoutput67_46e::Od_data7output_typeType od_data7output_type(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'OD_Data7Output_X', 'offset': -2048.0, 'precision': 1.0, 'len': 12, 'is_signed_var': False, 'physical_range': '[-2048|2047]', 'bit': 47, 'type': 'int', 'order': 'motorola', 'physical_unit': 'cm'}
  int od_data7output_x(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'OD_Data7Output_Y', 'offset': -2048.0, 'precision': 1.0, 'len': 12, 'is_signed_var': False, 'physical_range': '[-2048|2047]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': 'cm'}
  int od_data7output_y(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'OD_Data6Output_ConfidenceLvl', 'enum': {0: 'OD_DATA6OUTPUT_CONFIDENCELVL_LOW', 1: 'OD_DATA6OUTPUT_CONFIDENCELVL_GENERAL', 2: 'OD_DATA6OUTPUT_CONFIDENCELVL_BELIEVABLE', 3: 'OD_DATA6OUTPUT_CONFIDENCELVL_VERYBELIEVABLE', 4: 'OD_DATA6OUTPUT_CONFIDENCELVL_RESERVED'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 2, 'type': 'enum', 'order': 'motorola', 'physical_unit': '0'}
  Debug_oddataoutput67_46e::Od_data6output_confidencelvlType od_data6output_confidencelvl(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'OD_Data6Output_Type', 'enum': {0: 'OD_DATA6OUTPUT_TYPE_UNKNOWN', 1: 'OD_DATA6OUTPUT_TYPE_PEDESTRIAN', 2: 'OD_DATA6OUTPUT_TYPE_FOURWHEELEDVEHICLE', 3: 'OD_DATA6OUTPUT_TYPE_TWOWHEELEDVEHICLE', 4: 'OD_DATA6OUTPUT_TYPE_RIDER', 5: 'OD_DATA6OUTPUT_TYPE_GROUNDLOCK', 6: 'OD_DATA6OUTPUT_TYPE_CONE', 7: 'OD_DATA6OUTPUT_TYPE_WHEELBLOCK', 8: 'OD_DATA6OUTPUT_TYPE_WATERHORSE', 9: 'OD_DATA6OUTPUT_TYPE_RESERVED'}, 'precision': 1.0, 'len': 5, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|31]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': 'cm'}
  Debug_oddataoutput67_46e::Od_data6output_typeType od_data6output_type(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'OD_Data6Output_X', 'offset': -2048.0, 'precision': 1.0, 'len': 12, 'is_signed_var': False, 'physical_range': '[-2048|2047]', 'bit': 15, 'type': 'int', 'order': 'motorola', 'physical_unit': 'cm'}
  int od_data6output_x(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'OD_Data6Output_Y', 'offset': -2048.0, 'precision': 1.0, 'len': 12, 'is_signed_var': False, 'physical_range': '[-2048|2047]', 'bit': 19, 'type': 'int', 'order': 'motorola', 'physical_unit': 'cm'}
  int od_data6output_y(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_CX75_PROTOCOL_DEBUG_ODDATAOUTPUT67_46E_H_
