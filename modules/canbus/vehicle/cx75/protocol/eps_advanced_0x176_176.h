/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#ifndef MODULES_CANBUS_VEHICLE_cx75_PROTOCOL_EPS_ADVANCED_0X176_176_H_
#define MODULES_CANBUS_VEHICLE_cx75_PROTOCOL_EPS_ADVANCED_0X176_176_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

class Epsadvanced0x176176 : public ::jmc_auto::drivers::canbus::ProtocolData<
                    ::jmc_auto::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Epsadvanced0x176176();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'name': 'EPS_LKAResponseTorque', 'enum': {2047: 'EPS_LKARESPONSETORQUE_INVALID'}, 'precision': 0.01, 'len': 11, 'is_signed_var': False, 'offset': -10.24, 'physical_range': '[-10.24|10.22]', 'bit': 11, 'type': 'enum', 'order': 'motorola', 'physical_unit': 'Nm'}
  double eps_lkaresponsetorque(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'EPS_TosionBarTorqueValid', 'enum': {0: 'EPS_TOSIONBARTORQUEVALID_VAILD', 1: 'EPS_TOSIONBARTORQUEVALID_INVALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 12, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Eps_advanced_0x176_176::Eps_tosionbartorquevalidType eps_tosionbartorquevalid(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'EPS_LKAResponseTorqueValid', 'enum': {0: 'EPS_LKARESPONSETORQUEVALID_VAILD', 1: 'EPS_LKARESPONSETORQUEVALID_INVALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 16, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Eps_advanced_0x176_176::Eps_lkaresponsetorquevalidType eps_lkaresponsetorquevalid(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'EPS_LDWControlStatus', 'enum': {0: 'EPS_LDWCONTROLSTATUS_DEACTIVATED', 1: 'EPS_LDWCONTROLSTATUS_INACTIVE', 2: 'EPS_LDWCONTROLSTATUS_ACTIVE', 3: 'EPS_LDWCONTROLSTATUS_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 28, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Eps_advanced_0x176_176::Eps_ldwcontrolstatusType eps_ldwcontrolstatus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'EPS_PAM_SteeringSts_Reserved', 'enum': {0: 'EPS_PAM_STEERINGSTS_RESERVED_STEERING_POSITION_IS_OK', 1: 'EPS_PAM_STEERINGSTS_RESERVED_STEERING_POSITION_IS_ADJUSTING'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 29, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Eps_advanced_0x176_176::Eps_pam_steeringsts_reservedType eps_pam_steeringsts_reserved(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'EPS_LKAControlStatus', 'enum': {0: 'EPS_LKACONTROLSTATUS_NO_REQUEST', 1: 'EPS_LKACONTROLSTATUS_REQUEST_HONORED', 2: 'EPS_LKACONTROLSTATUS_CONTROL_REQUEST_NOT_ALLOWED_TEMPORARILY', 3: 'EPS_LKACONTROLSTATUS_CONTROL_REQUEST_NOT_ALLOWED_PERMANENT'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 31, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Eps_advanced_0x176_176::Eps_lkacontrolstatusType eps_lkacontrolstatus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'EPS_EPSPAMInh', 'enum': {0: 'EPS_EPSPAMINH_NORMAL_OPERATION', 1: 'EPS_EPSPAMINH_OVER_SPEED', 2: 'EPS_EPSPAMINH_DRIVER_INTERFERENCE', 4: 'EPS_EPSPAMINH_ABNORMAL_CAN_INPUT', 16: 'EPS_EPSPAMINH_EPS_FAILURE', 8: 'EPS_EPSPAMINH_EXCESS_ANGLE_DEVIATION'}, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|255]', 'bit': 39, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  //Eps_advanced_0x176_176::Eps_epspaminhType eps_epspaminh(const std::uint8_t* bytes, const int32_t length) const;
  
  int eps_epspaminh(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Rolling_counter_0x176', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int rolling_counter_0x176(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'EPS_EPSPAMSts', 'enum': {0: 'EPS_EPSPAMSTS_INHIBIT', 1: 'EPS_EPSPAMSTS_AVAILABLE', 2: 'EPS_EPSPAMSTS_ACTIVE', 3: 'EPS_EPSPAMSTS_ABORT', 4: 'EPS_EPSPAMSTS_REPLY_FOR_CONTROL', 5: 'EPS_EPSPAMSTS_NRCD_ACTIVE', 6: 'EPS_EPSPAMSTS_ADAS_ACTIVE', 7: 'EPS_EPSPAMSTS_FAILURE'}, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|15]', 'bit': 55, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Eps_advanced_0x176_176::Eps_epspamstsType eps_epspamsts(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Checksum_0x176', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int checksum_0x176(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'EPS_TorsionBarTorque', 'offset': -10.24, 'precision': 0.01, 'len': 11, 'is_signed_var': False, 'physical_range': '[-10.24|10.22]', 'bit': 7, 'type': 'double', 'order': 'motorola', 'physical_unit': 'Nm'}
  double eps_torsionbartorque(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_cx75_PROTOCOL_EPS_ADVANCED_0X176_176_H_
