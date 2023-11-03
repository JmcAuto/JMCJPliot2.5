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

#ifndef MODULES_DRIVERS_RADAR_TTE_CONTI_RADAR_PROTOCOL_DEBUG_PARKINGTYPE_533_H_
#define MODULES_DRIVERS_RADAR_TTE_CONTI_RADAR_PROTOCOL_DEBUG_PARKINGTYPE_533_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/drivers/proto/tte_conti_radar.pb.h"

namespace jmc_auto {
namespace drivers {
namespace tte_conti_radar {

class Debugparkingtype533 : public ::jmc_auto::drivers::canbus::ProtocolData<
                    ::jmc_auto::drivers::TteContiRadar> {
 public:
  static const int32_t ID;

  Debugparkingtype533();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'name': 'ParkingType533_Copy_7', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  Debugparkingtype533* set_parkingtype533_copy_7(int parkingtype533_copy_7);

  // config detail: {'name': 'ParkingType533_Copy_6', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 55, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  Debugparkingtype533* set_parkingtype533_copy_6(int parkingtype533_copy_6);

  // config detail: {'name': 'ParkingType533_Copy_5', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 47, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  Debugparkingtype533* set_parkingtype533_copy_5(int parkingtype533_copy_5);

  // config detail: {'name': 'ParkingType533_Copy_4', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 39, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  Debugparkingtype533* set_parkingtype533_copy_4(int parkingtype533_copy_4);

  // config detail: {'name': 'ParkingType533_Copy_3', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 31, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  Debugparkingtype533* set_parkingtype533_copy_3(int parkingtype533_copy_3);

  // config detail: {'name': 'ParkingType533_Copy_2', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 23, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  Debugparkingtype533* set_parkingtype533_copy_2(int parkingtype533_copy_2);

  // config detail: {'name': 'ParkingType533_Copy_1', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 15, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  Debugparkingtype533* set_parkingtype533_copy_1(int parkingtype533_copy_1);

  // config detail: {'name': 'ParkingType533', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 7, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  Debugparkingtype533* set_parkingtype533(int parkingtype533);

 private:

  // config detail: {'name': 'ParkingType533_Copy_7', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  void set_p_parkingtype533_copy_7(uint8_t* data, int parkingtype533_copy_7);

  // config detail: {'name': 'ParkingType533_Copy_6', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 55, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  void set_p_parkingtype533_copy_6(uint8_t* data, int parkingtype533_copy_6);

  // config detail: {'name': 'ParkingType533_Copy_5', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 47, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  void set_p_parkingtype533_copy_5(uint8_t* data, int parkingtype533_copy_5);

  // config detail: {'name': 'ParkingType533_Copy_4', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 39, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  void set_p_parkingtype533_copy_4(uint8_t* data, int parkingtype533_copy_4);

  // config detail: {'name': 'ParkingType533_Copy_3', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 31, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  void set_p_parkingtype533_copy_3(uint8_t* data, int parkingtype533_copy_3);

  // config detail: {'name': 'ParkingType533_Copy_2', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 23, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  void set_p_parkingtype533_copy_2(uint8_t* data, int parkingtype533_copy_2);

  // config detail: {'name': 'ParkingType533_Copy_1', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 15, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  void set_p_parkingtype533_copy_1(uint8_t* data, int parkingtype533_copy_1);

  // config detail: {'name': 'ParkingType533', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 7, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  void set_p_parkingtype533(uint8_t* data, int parkingtype533);

 private:
  int parkingtype533_copy_7_;
  int parkingtype533_copy_6_;
  int parkingtype533_copy_5_;
  int parkingtype533_copy_4_;
  int parkingtype533_copy_3_;
  int parkingtype533_copy_2_;
  int parkingtype533_copy_1_;
  int parkingtype533_;
};

}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_CX75_PROTOCOL_DEBUG_PARKINGTYPE_533_H_
