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

#ifndef MODULES_CANBUS_VEHICLE_TESHUN_PROTOCOL_ADU_CONTROLDRIVE_0X120_120_H_
#define MODULES_CANBUS_VEHICLE_TESHUN_PROTOCOL_ADU_CONTROLDRIVE_0X120_120_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace jmc_auto {
namespace canbus {
namespace teshun {

class Aducontroldrive0x120120 : public ::jmc_auto::drivers::canbus::ProtocolData<
                    ::jmc_auto::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Aducontroldrive0x120120();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'name': 'Checksum_0x120', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  Aducontroldrive0x120120* set_checksum_0x120(int checksum_0x120);

  // config detail: {'name': 'Rolling_counter_0x120', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  Aducontroldrive0x120120* set_rolling_counter_0x120(int rolling_counter_0x120);

  // config detail: {'name': 'ADU_ControTorque_RAWFlag', 'enum': {0: 'ADU_CONTROTORQUE_RAWFLAG_NO_FILTERS', 1: 'ADU_CONTROTORQUE_RAWFLAG_FILTERS'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 0, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Aducontroldrive0x120120* set_adu_controtorque_rawflag(Adu_controldrive_0x120_120::Adu_controtorque_rawflagType adu_controtorque_rawflag);

  // config detail: {'name': 'ADU_TargetDrving_Torque', 'offset': -10000.0, 'precision': 1.0, 'len': 16, 'is_signed_var': False, 'physical_range': '[-10000|10000]', 'bit': 15, 'type': 'int', 'order': 'motorola', 'physical_unit': 'Nm'}
  Aducontroldrive0x120120* set_adu_targetdrving_torque(int adu_targetdrving_torque);

  // config detail: {'name': 'ADU_ControTorque_Enable', 'enum': {0: 'ADU_CONTROTORQUE_ENABLE_DISABLE', 1: 'ADU_CONTROTORQUE_ENABLE_ENABLE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 3, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Aducontroldrive0x120120* set_adu_controtorque_enable(Adu_controldrive_0x120_120::Adu_controtorque_enableType adu_controtorque_enable);

  // config detail: {'name': 'ADU_TargetGear_Position', 'enum': {0: 'ADU_TARGETGEAR_POSITION_INITIAL', 1: 'ADU_TARGETGEAR_POSITION_P_PARK', 2: 'ADU_TARGETGEAR_POSITION_R_REVERSE', 3: 'ADU_TARGETGEAR_POSITION_N_NEUTRAL', 4: 'ADU_TARGETGEAR_POSITION_D_DRIVE', 5: 'ADU_TARGETGEAR_POSITION_INVALID'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 6, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Aducontroldrive0x120120* set_adu_targetgear_position(Adu_controldrive_0x120_120::Adu_targetgear_positionType adu_targetgear_position);

  // config detail: {'name': 'ADU_TargetGear_Enable', 'enum': {0: 'ADU_TARGETGEAR_ENABLE_DISABLE', 1: 'ADU_TARGETGEAR_ENABLE_ENABLE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Aducontroldrive0x120120* set_adu_targetgear_enable(Adu_controldrive_0x120_120::Adu_targetgear_enableType adu_targetgear_enable);

 private:

  // config detail: {'name': 'Checksum_0x120', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  void set_p_checksum_0x120(uint8_t* data, int checksum_0x120);

  // config detail: {'name': 'Rolling_counter_0x120', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  void set_p_rolling_counter_0x120(uint8_t* data, int rolling_counter_0x120);

  // config detail: {'name': 'ADU_ControTorque_RAWFlag', 'enum': {0: 'ADU_CONTROTORQUE_RAWFLAG_NO_FILTERS', 1: 'ADU_CONTROTORQUE_RAWFLAG_FILTERS'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 0, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_adu_controtorque_rawflag(uint8_t* data, Adu_controldrive_0x120_120::Adu_controtorque_rawflagType adu_controtorque_rawflag);

  // config detail: {'name': 'ADU_TargetDrving_Torque', 'offset': -10000.0, 'precision': 1.0, 'len': 16, 'is_signed_var': False, 'physical_range': '[-10000|10000]', 'bit': 15, 'type': 'int', 'order': 'motorola', 'physical_unit': 'Nm'}
  void set_p_adu_targetdrving_torque(uint8_t* data, int adu_targetdrving_torque);

  // config detail: {'name': 'ADU_ControTorque_Enable', 'enum': {0: 'ADU_CONTROTORQUE_ENABLE_DISABLE', 1: 'ADU_CONTROTORQUE_ENABLE_ENABLE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 3, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_adu_controtorque_enable(uint8_t* data, Adu_controldrive_0x120_120::Adu_controtorque_enableType adu_controtorque_enable);

  // config detail: {'name': 'ADU_TargetGear_Position', 'enum': {0: 'ADU_TARGETGEAR_POSITION_INITIAL', 1: 'ADU_TARGETGEAR_POSITION_P_PARK', 2: 'ADU_TARGETGEAR_POSITION_R_REVERSE', 3: 'ADU_TARGETGEAR_POSITION_N_NEUTRAL', 4: 'ADU_TARGETGEAR_POSITION_D_DRIVE', 5: 'ADU_TARGETGEAR_POSITION_INVALID'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 6, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_adu_targetgear_position(uint8_t* data, Adu_controldrive_0x120_120::Adu_targetgear_positionType adu_targetgear_position);

  // config detail: {'name': 'ADU_TargetGear_Enable', 'enum': {0: 'ADU_TARGETGEAR_ENABLE_DISABLE', 1: 'ADU_TARGETGEAR_ENABLE_ENABLE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_adu_targetgear_enable(uint8_t* data, Adu_controldrive_0x120_120::Adu_targetgear_enableType adu_targetgear_enable);

 private:
  int checksum_0x120_;
  int rolling_counter_0x120_;
  Adu_controldrive_0x120_120::Adu_controtorque_rawflagType adu_controtorque_rawflag_;
  int adu_targetdrving_torque_;
  Adu_controldrive_0x120_120::Adu_controtorque_enableType adu_controtorque_enable_;
  Adu_controldrive_0x120_120::Adu_targetgear_positionType adu_targetgear_position_;
  Adu_controldrive_0x120_120::Adu_targetgear_enableType adu_targetgear_enable_;
};

}  // namespace teshun
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_TESHUN_PROTOCOL_ADU_CONTROLDRIVE_0X120_120_H_
