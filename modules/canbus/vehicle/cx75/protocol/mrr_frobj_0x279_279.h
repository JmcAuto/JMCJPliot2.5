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

#ifndef MODULES_CANBUS_VEHICLE_cx75_PROTOCOL_MRR_FROBJ_0X279_279_H_
#define MODULES_CANBUS_VEHICLE_cx75_PROTOCOL_MRR_FROBJ_0X279_279_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

class Mrrfrobj0x279279 : public ::jmc_auto::drivers::canbus::ProtocolData<
                    ::jmc_auto::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Mrrfrobj0x279279();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'name': 'MRR_F_Object_dy', 'enum': {511: 'MRR_F_OBJECT_DY_NO_OBJECT'}, 'precision': 0.0625, 'len': 9, 'is_signed_var': False, 'offset': -15.0, 'physical_range': '[-15|15]', 'bit': 11, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Mrrfrobj0x279279* set_mrr_f_object_dy(double mrr_f_object_dy);

  // config detail: {'name': 'MRR_F_Obj_0x_class', 'enum': {0: 'MRR_F_OBJ_0X_CLASS_UNKNOWN', 1: 'MRR_F_OBJ_0X_CLASS_CAR', 2: 'MRR_F_OBJ_0X_CLASS_TRUCK', 3: 'MRR_F_OBJ_0X_CLASS_TWO_WHEELER'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 18, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Mrrfrobj0x279279* set_mrr_f_obj_0x_class(Mrr_frobj_0x279_279::Mrr_f_obj_0x_classType mrr_f_obj_0x_class);

  // config detail: {'name': 'MRR_FF_Object_dx', 'enum': {4095: 'MRR_FF_OBJECT_DX_NO_OBJECT'}, 'precision': 0.0625, 'len': 12, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|255.875]', 'bit': 31, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Mrrfrobj0x279279* set_mrr_ff_object_dx(double mrr_ff_object_dx);

  // config detail: {'name': 'MRR_FF_Object_dy', 'enum': {511: 'MRR_FF_OBJECT_DY_NO_OBJECT'}, 'precision': 0.0625, 'len': 9, 'is_signed_var': False, 'offset': -15.0, 'physical_range': '[-15|15]', 'bit': 35, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Mrrfrobj0x279279* set_mrr_ff_object_dy(double mrr_ff_object_dy);

  // config detail: {'name': 'MRR_FF_Obj_0x_class', 'enum': {0: 'MRR_FF_OBJ_0X_CLASS_UNKNOWN', 1: 'MRR_FF_OBJ_0X_CLASS_CAR', 2: 'MRR_FF_OBJ_0X_CLASS_TRUCK', 3: 'MRR_FF_OBJ_0X_CLASS_TWO_WHEELER'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 42, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Mrrfrobj0x279279* set_mrr_ff_obj_0x_class(Mrr_frobj_0x279_279::Mrr_ff_obj_0x_classType mrr_ff_obj_0x_class);

  // config detail: {'name': 'MRR_FFTargrtDetection', 'enum': {0: 'MRR_FFTARGRTDETECTION_NOT_DECTECTED', 1: 'MRR_FFTARGRTDETECTION_DECTECTED'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 54, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Mrrfrobj0x279279* set_mrr_fftargrtdetection(Mrr_frobj_0x279_279::Mrr_fftargrtdetectionType mrr_fftargrtdetection);

  // config detail: {'name': 'MRR_PedDetection', 'enum': {0: 'MRR_PEDDETECTION_NOT_DECTECTED', 1: 'MRR_PEDDETECTION_DECTECTED'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 55, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Mrrfrobj0x279279* set_mrr_peddetection(Mrr_frobj_0x279_279::Mrr_peddetectionType mrr_peddetection);

  // config detail: {'name': 'MRR_F_Object_dx', 'enum': {4095: 'MRR_F_OBJECT_DX_NO_OBJECT'}, 'precision': 0.0625, 'len': 12, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|255.875]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Mrrfrobj0x279279* set_mrr_f_object_dx(double mrr_f_object_dx);

 private:

  // config detail: {'name': 'MRR_F_Object_dy', 'enum': {511: 'MRR_F_OBJECT_DY_NO_OBJECT'}, 'precision': 0.0625, 'len': 9, 'is_signed_var': False, 'offset': -15.0, 'physical_range': '[-15|15]', 'bit': 11, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_mrr_f_object_dy(uint8_t* data, double mrr_f_object_dy);

  // config detail: {'name': 'MRR_F_Obj_0x_class', 'enum': {0: 'MRR_F_OBJ_0X_CLASS_UNKNOWN', 1: 'MRR_F_OBJ_0X_CLASS_CAR', 2: 'MRR_F_OBJ_0X_CLASS_TRUCK', 3: 'MRR_F_OBJ_0X_CLASS_TWO_WHEELER'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 18, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_mrr_f_obj_0x_class(uint8_t* data, Mrr_frobj_0x279_279::Mrr_f_obj_0x_classType mrr_f_obj_0x_class);

  // config detail: {'name': 'MRR_FF_Object_dx', 'enum': {4095: 'MRR_FF_OBJECT_DX_NO_OBJECT'}, 'precision': 0.0625, 'len': 12, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|255.875]', 'bit': 31, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_mrr_ff_object_dx(uint8_t* data, double mrr_ff_object_dx);

  // config detail: {'name': 'MRR_FF_Object_dy', 'enum': {511: 'MRR_FF_OBJECT_DY_NO_OBJECT'}, 'precision': 0.0625, 'len': 9, 'is_signed_var': False, 'offset': -15.0, 'physical_range': '[-15|15]', 'bit': 35, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_mrr_ff_object_dy(uint8_t* data, double mrr_ff_object_dy);

  // config detail: {'name': 'MRR_FF_Obj_0x_class', 'enum': {0: 'MRR_FF_OBJ_0X_CLASS_UNKNOWN', 1: 'MRR_FF_OBJ_0X_CLASS_CAR', 2: 'MRR_FF_OBJ_0X_CLASS_TRUCK', 3: 'MRR_FF_OBJ_0X_CLASS_TWO_WHEELER'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 42, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_mrr_ff_obj_0x_class(uint8_t* data, Mrr_frobj_0x279_279::Mrr_ff_obj_0x_classType mrr_ff_obj_0x_class);

  // config detail: {'name': 'MRR_FFTargrtDetection', 'enum': {0: 'MRR_FFTARGRTDETECTION_NOT_DECTECTED', 1: 'MRR_FFTARGRTDETECTION_DECTECTED'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 54, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_mrr_fftargrtdetection(uint8_t* data, Mrr_frobj_0x279_279::Mrr_fftargrtdetectionType mrr_fftargrtdetection);

  // config detail: {'name': 'MRR_PedDetection', 'enum': {0: 'MRR_PEDDETECTION_NOT_DECTECTED', 1: 'MRR_PEDDETECTION_DECTECTED'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 55, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_mrr_peddetection(uint8_t* data, Mrr_frobj_0x279_279::Mrr_peddetectionType mrr_peddetection);

  // config detail: {'name': 'MRR_F_Object_dx', 'enum': {4095: 'MRR_F_OBJECT_DX_NO_OBJECT'}, 'precision': 0.0625, 'len': 12, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|255.875]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_mrr_f_object_dx(uint8_t* data, double mrr_f_object_dx);

 private:
  double mrr_f_object_dy_;
  Mrr_frobj_0x279_279::Mrr_f_obj_0x_classType mrr_f_obj_0x_class_;
  double mrr_ff_object_dx_;
  double mrr_ff_object_dy_;
  Mrr_frobj_0x279_279::Mrr_ff_obj_0x_classType mrr_ff_obj_0x_class_;
  Mrr_frobj_0x279_279::Mrr_fftargrtdetectionType mrr_fftargrtdetection_;
  Mrr_frobj_0x279_279::Mrr_peddetectionType mrr_peddetection_;
  double mrr_f_object_dx_;
};

}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_cx75_PROTOCOL_MRR_FROBJ_0X279_279_H_
