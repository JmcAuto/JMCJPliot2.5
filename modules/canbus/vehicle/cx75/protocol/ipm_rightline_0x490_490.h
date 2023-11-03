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

#ifndef MODULES_CANBUS_VEHICLE_cx75_PROTOCOL_IPM_RIGHTLINE_0X490_490_H_
#define MODULES_CANBUS_VEHICLE_cx75_PROTOCOL_IPM_RIGHTLINE_0X490_490_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

class Ipmrightline0x490490 : public ::jmc_auto::drivers::canbus::ProtocolData<
                    ::jmc_auto::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Ipmrightline0x490490();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'name': 'IPM_RightLine_dy', 'enum': {0: 'IPM_RIGHTLINE_DY_INITIAL_VALUE_NO_LINE'}, 'precision': 0.00390625, 'len': 12, 'is_signed_var': False, 'offset': -8.0, 'physical_range': '[-8|7.99609375]', 'bit': 15, 'type': 'enum', 'order': 'motorola', 'physical_unit': 'm'}
  Ipmrightline0x490490* set_ipm_rightline_dy(double ipm_rightline_dy);

  // config detail: {'name': 'IPM_RightLine_dx_lookhead', 'offset': 0.0, 'precision': 0.25, 'len': 9, 'is_signed_var': False, 'physical_range': '[0|127.75]', 'bit': 19, 'type': 'double', 'order': 'motorola', 'physical_unit': 'm'}
  Ipmrightline0x490490* set_ipm_rightline_dx_lookhead(double ipm_rightline_dx_lookhead);

  // config detail: {'name': 'IPM_RightLine_hor_curve', 'enum': {0: 'IPM_RIGHTLINE_HOR_CURVE_INITIAL_VALUE_NO_CURVATURE'}, 'precision': 0.0001, 'len': 10, 'is_signed_var': False, 'offset': -0.015, 'physical_range': '[-0.015|0.0513]', 'bit': 26, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Ipmrightline0x490490* set_ipm_rightline_hor_curve(double ipm_rightline_hor_curve);

  // config detail: {'name': 'IPM_RightLine_yawangle', 'offset': -1.0, 'precision': 0.000488281, 'len': 12, 'is_signed_var': False, 'physical_range': '[-1|0.999510695]', 'bit': 47, 'type': 'double', 'order': 'motorola', 'physical_unit': ''}
  Ipmrightline0x490490* set_ipm_rightline_yawangle(double ipm_rightline_yawangle);

  // config detail: {'name': 'IPM_RightLine_dx_start', 'offset': 0.0, 'precision': 0.25, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|63.75]', 'bit': 7, 'type': 'double', 'order': 'motorola', 'physical_unit': 'm'}
  Ipmrightline0x490490* set_ipm_rightline_dx_start(double ipm_rightline_dx_start);

 private:

  // config detail: {'name': 'IPM_RightLine_dy', 'enum': {0: 'IPM_RIGHTLINE_DY_INITIAL_VALUE_NO_LINE'}, 'precision': 0.00390625, 'len': 12, 'is_signed_var': False, 'offset': -8.0, 'physical_range': '[-8|7.99609375]', 'bit': 15, 'type': 'enum', 'order': 'motorola', 'physical_unit': 'm'}
  void set_p_ipm_rightline_dy(uint8_t* data, double ipm_rightline_dy);

  // config detail: {'name': 'IPM_RightLine_dx_lookhead', 'offset': 0.0, 'precision': 0.25, 'len': 9, 'is_signed_var': False, 'physical_range': '[0|127.75]', 'bit': 19, 'type': 'double', 'order': 'motorola', 'physical_unit': 'm'}
  void set_p_ipm_rightline_dx_lookhead(uint8_t* data, double ipm_rightline_dx_lookhead);

  // config detail: {'name': 'IPM_RightLine_hor_curve', 'enum': {0: 'IPM_RIGHTLINE_HOR_CURVE_INITIAL_VALUE_NO_CURVATURE'}, 'precision': 0.0001, 'len': 10, 'is_signed_var': False, 'offset': -0.015, 'physical_range': '[-0.015|0.0513]', 'bit': 26, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_ipm_rightline_hor_curve(uint8_t* data, double ipm_rightline_hor_curve);

  // config detail: {'name': 'IPM_RightLine_yawangle', 'offset': -1.0, 'precision': 0.000488281, 'len': 12, 'is_signed_var': False, 'physical_range': '[-1|0.999510695]', 'bit': 47, 'type': 'double', 'order': 'motorola', 'physical_unit': ''}
  void set_p_ipm_rightline_yawangle(uint8_t* data, double ipm_rightline_yawangle);

  // config detail: {'name': 'IPM_RightLine_dx_start', 'offset': 0.0, 'precision': 0.25, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|63.75]', 'bit': 7, 'type': 'double', 'order': 'motorola', 'physical_unit': 'm'}
  void set_p_ipm_rightline_dx_start(uint8_t* data, double ipm_rightline_dx_start);

 private:
  double ipm_rightline_dy_;
  double ipm_rightline_dx_lookhead_;
  double ipm_rightline_hor_curve_;
  double ipm_rightline_yawangle_;
  double ipm_rightline_dx_start_;
};

}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_cx75_PROTOCOL_IPM_RIGHTLINE_0X490_490_H_
