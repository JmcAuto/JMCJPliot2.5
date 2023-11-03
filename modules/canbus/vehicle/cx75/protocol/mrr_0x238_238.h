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

#ifndef MODULES_CANBUS_VEHICLE_cx75_PROTOCOL_MRR_0X238_238_H_
#define MODULES_CANBUS_VEHICLE_cx75_PROTOCOL_MRR_0X238_238_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

class Mrr0x238238 : public ::jmc_auto::drivers::canbus::ProtocolData<
                    ::jmc_auto::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Mrr0x238238();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;
  
  void checksum_rolling();

  // config detail: {'name': 'ACC_TgtAxLowerComftBand', 'offset': 0.0, 'precision': 0.05, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|3.2]', 'bit': 15, 'type': 'double', 'order': 'motorola', 'physical_unit': 'm s2'}
  Mrr0x238238* set_acc_tgtaxlowercomftband(double acc_tgtaxlowercomftband);

  // config detail: {'name': 'ACC_TgtAxUpperLim', 'offset': 0.0, 'precision': 0.1, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|12.7]', 'bit': 23, 'type': 'double', 'order': 'motorola', 'physical_unit': 'm s3'}
  Mrr0x238238* set_acc_tgtaxupperlim(double acc_tgtaxupperlim);

  // config detail: {'name': 'ACC_TgtAxLowerLim', 'offset': -12.7, 'precision': 0.1, 'len': 8, 'is_signed_var': False, 'physical_range': '[-12.7|0]', 'bit': 31, 'type': 'double', 'order': 'motorola', 'physical_unit': 'm s3'}
  Mrr0x238238* set_acc_tgtaxlowerlim(double acc_tgtaxlowerlim);

  // config detail: {'name': 'ACC_TgtAx', 'offset': -5.0, 'precision': 0.05, 'len': 11, 'is_signed_var': False, 'physical_range': '[-5|7.75]', 'bit': 47, 'type': 'double', 'order': 'motorola', 'physical_unit': 'm s2'}
  Mrr0x238238* set_acc_tgtax(double acc_tgtax);

  // config detail: {'name': 'Rolling_counter_0x238', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  Mrr0x238238* set_rolling_counter_0x238(int rolling_counter_0x238);

  // config detail: {'name': 'Checksum_0x238', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  Mrr0x238238* set_checksum_0x238(int checksum_0x238);

  // config detail: {'name': 'ACC_TgtAxUpperComftBand', 'offset': 0.0, 'precision': 0.05, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|3.2]', 'bit': 7, 'type': 'double', 'order': 'motorola', 'physical_unit': 'm s2'}
  Mrr0x238238* set_acc_tgtaxuppercomftband(double acc_tgtaxuppercomftband);

 private:

  // config detail: {'name': 'ACC_TgtAxLowerComftBand', 'offset': 0.0, 'precision': 0.05, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|3.2]', 'bit': 15, 'type': 'double', 'order': 'motorola', 'physical_unit': 'm s2'}
  void set_p_acc_tgtaxlowercomftband(uint8_t* data, double acc_tgtaxlowercomftband);

  // config detail: {'name': 'ACC_TgtAxUpperLim', 'offset': 0.0, 'precision': 0.1, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|12.7]', 'bit': 23, 'type': 'double', 'order': 'motorola', 'physical_unit': 'm s3'}
  void set_p_acc_tgtaxupperlim(uint8_t* data, double acc_tgtaxupperlim);

  // config detail: {'name': 'ACC_TgtAxLowerLim', 'offset': -12.7, 'precision': 0.1, 'len': 8, 'is_signed_var': False, 'physical_range': '[-12.7|0]', 'bit': 31, 'type': 'double', 'order': 'motorola', 'physical_unit': 'm s3'}
  void set_p_acc_tgtaxlowerlim(uint8_t* data, double acc_tgtaxlowerlim);

  // config detail: {'name': 'ACC_TgtAx', 'offset': -5.0, 'precision': 0.05, 'len': 11, 'is_signed_var': False, 'physical_range': '[-5|7.75]', 'bit': 47, 'type': 'double', 'order': 'motorola', 'physical_unit': 'm s2'}
  void set_p_acc_tgtax(uint8_t* data, double acc_tgtax);

  // config detail: {'name': 'Rolling_counter_0x238', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  void set_p_rolling_counter_0x238(uint8_t* data, int rolling_counter_0x238);

  // config detail: {'name': 'Checksum_0x238', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  void set_p_checksum_0x238(uint8_t* data, int checksum_0x238);

  // config detail: {'name': 'ACC_TgtAxUpperComftBand', 'offset': 0.0, 'precision': 0.05, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|3.2]', 'bit': 7, 'type': 'double', 'order': 'motorola', 'physical_unit': 'm s2'}
  void set_p_acc_tgtaxuppercomftband(uint8_t* data, double acc_tgtaxuppercomftband);

 private:
  double acc_tgtaxlowercomftband_;
  double acc_tgtaxupperlim_;
  double acc_tgtaxlowerlim_;
  double acc_tgtax_;
  int rolling_counter_0x238_;
  int checksum_0x238_;
  double acc_tgtaxuppercomftband_;
};

}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_cx75_PROTOCOL_MRR_0X238_238_H_
