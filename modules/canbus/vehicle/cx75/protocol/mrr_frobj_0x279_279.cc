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

#include "modules/canbus/vehicle/cx75/protocol/mrr_frobj_0x279_279.h"

#include "modules/drivers/canbus/common/byte.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

using ::jmc_auto::drivers::canbus::Byte;

const int32_t Mrrfrobj0x279279::ID = 0x279;

// public
Mrrfrobj0x279279::Mrrfrobj0x279279() { Reset(); }

uint32_t Mrrfrobj0x279279::GetPeriod() const {
  // TODO modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Mrrfrobj0x279279::UpdateData(uint8_t* data) {
  set_p_mrr_f_object_dy(data, mrr_f_object_dy_);
  set_p_mrr_f_obj_0x_class(data, mrr_f_obj_0x_class_);
  set_p_mrr_ff_object_dx(data, mrr_ff_object_dx_);
  set_p_mrr_ff_object_dy(data, mrr_ff_object_dy_);
  set_p_mrr_ff_obj_0x_class(data, mrr_ff_obj_0x_class_);
  set_p_mrr_fftargrtdetection(data, mrr_fftargrtdetection_);
  set_p_mrr_peddetection(data, mrr_peddetection_);
  set_p_mrr_f_object_dx(data, mrr_f_object_dx_);
}

void Mrrfrobj0x279279::Reset() {
  // TODO you should check this manually
  mrr_f_object_dy_ = 0.0;
  mrr_f_obj_0x_class_ = Mrr_frobj_0x279_279::MRR_F_OBJ_0X_CLASS_UNKNOWN;
  mrr_ff_object_dx_ = 0.0;
  mrr_ff_object_dy_ = 0.0;
  mrr_ff_obj_0x_class_ = Mrr_frobj_0x279_279::MRR_FF_OBJ_0X_CLASS_UNKNOWN;
  mrr_fftargrtdetection_ = Mrr_frobj_0x279_279::MRR_FFTARGRTDETECTION_NOT_DECTECTED;
  mrr_peddetection_ = Mrr_frobj_0x279_279::MRR_PEDDETECTION_NOT_DECTECTED;
  mrr_f_object_dx_ = 0.0;
}

Mrrfrobj0x279279* Mrrfrobj0x279279::set_mrr_f_object_dy(
    double mrr_f_object_dy) {
  mrr_f_object_dy_ = mrr_f_object_dy;
  return this;
 }

// config detail: {'name': 'MRR_F_Object_dy', 'enum': {511: 'MRR_F_OBJECT_DY_NO_OBJECT'}, 'precision': 0.0625, 'len': 9, 'is_signed_var': False, 'offset': -15.0, 'physical_range': '[-15|15]', 'bit': 11, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Mrrfrobj0x279279::set_p_mrr_f_object_dy(uint8_t* data,
    double mrr_f_object_dy) {
  int x = (mrr_f_object_dy - -15.000000) / 0.062500;
  uint8_t t = 0;

  t = x & 0x1F;
  Byte to_set0(data + 2);
  to_set0.set_value(t, 3, 5);
  x >>= 5;

  t = x & 0xF;
  Byte to_set1(data + 1);
  to_set1.set_value(t, 0, 4);
}


Mrrfrobj0x279279* Mrrfrobj0x279279::set_mrr_f_obj_0x_class(
    Mrr_frobj_0x279_279::Mrr_f_obj_0x_classType mrr_f_obj_0x_class) {
  mrr_f_obj_0x_class_ = mrr_f_obj_0x_class;
  return this;
 }

// config detail: {'name': 'MRR_F_Obj_0x_class', 'enum': {0: 'MRR_F_OBJ_0X_CLASS_UNKNOWN', 1: 'MRR_F_OBJ_0X_CLASS_CAR', 2: 'MRR_F_OBJ_0X_CLASS_TRUCK', 3: 'MRR_F_OBJ_0X_CLASS_TWO_WHEELER'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 18, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Mrrfrobj0x279279::set_p_mrr_f_obj_0x_class(uint8_t* data,
    Mrr_frobj_0x279_279::Mrr_f_obj_0x_classType mrr_f_obj_0x_class) {
  int x = mrr_f_obj_0x_class;

  Byte to_set(data + 2);
  to_set.set_value(x, 0, 3);
}


Mrrfrobj0x279279* Mrrfrobj0x279279::set_mrr_ff_object_dx(
    double mrr_ff_object_dx) {
  mrr_ff_object_dx_ = mrr_ff_object_dx;
  return this;
 }

// config detail: {'name': 'MRR_FF_Object_dx', 'enum': {4095: 'MRR_FF_OBJECT_DX_NO_OBJECT'}, 'precision': 0.0625, 'len': 12, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|255.875]', 'bit': 31, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Mrrfrobj0x279279::set_p_mrr_ff_object_dx(uint8_t* data,
    double mrr_ff_object_dx) {
  int x = mrr_ff_object_dx / 0.062500;
  uint8_t t = 0;

  t = x & 0xF;
  Byte to_set0(data + 4);
  to_set0.set_value(t, 4, 4);
  x >>= 4;

  t = x & 0xFF;
  Byte to_set1(data + 3);
  to_set1.set_value(t, 0, 8);
}


Mrrfrobj0x279279* Mrrfrobj0x279279::set_mrr_ff_object_dy(
    double mrr_ff_object_dy) {
  mrr_ff_object_dy_ = mrr_ff_object_dy;
  return this;
 }

// config detail: {'name': 'MRR_FF_Object_dy', 'enum': {511: 'MRR_FF_OBJECT_DY_NO_OBJECT'}, 'precision': 0.0625, 'len': 9, 'is_signed_var': False, 'offset': -15.0, 'physical_range': '[-15|15]', 'bit': 35, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Mrrfrobj0x279279::set_p_mrr_ff_object_dy(uint8_t* data,
    double mrr_ff_object_dy) {
  int x = (mrr_ff_object_dy - -15.000000) / 0.062500;
  uint8_t t = 0;

  t = x & 0x1F;
  Byte to_set0(data + 5);
  to_set0.set_value(t, 3, 5);
  x >>= 5;

  t = x & 0xF;
  Byte to_set1(data + 4);
  to_set1.set_value(t, 0, 4);
}


Mrrfrobj0x279279* Mrrfrobj0x279279::set_mrr_ff_obj_0x_class(
    Mrr_frobj_0x279_279::Mrr_ff_obj_0x_classType mrr_ff_obj_0x_class) {
  mrr_ff_obj_0x_class_ = mrr_ff_obj_0x_class;
  return this;
 }

// config detail: {'name': 'MRR_FF_Obj_0x_class', 'enum': {0: 'MRR_FF_OBJ_0X_CLASS_UNKNOWN', 1: 'MRR_FF_OBJ_0X_CLASS_CAR', 2: 'MRR_FF_OBJ_0X_CLASS_TRUCK', 3: 'MRR_FF_OBJ_0X_CLASS_TWO_WHEELER'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 42, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Mrrfrobj0x279279::set_p_mrr_ff_obj_0x_class(uint8_t* data,
    Mrr_frobj_0x279_279::Mrr_ff_obj_0x_classType mrr_ff_obj_0x_class) {
  int x = mrr_ff_obj_0x_class;

  Byte to_set(data + 5);
  to_set.set_value(x, 0, 3);
}


Mrrfrobj0x279279* Mrrfrobj0x279279::set_mrr_fftargrtdetection(
    Mrr_frobj_0x279_279::Mrr_fftargrtdetectionType mrr_fftargrtdetection) {
  mrr_fftargrtdetection_ = mrr_fftargrtdetection;
  return this;
 }

// config detail: {'name': 'MRR_FFTargrtDetection', 'enum': {0: 'MRR_FFTARGRTDETECTION_NOT_DECTECTED', 1: 'MRR_FFTARGRTDETECTION_DECTECTED'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 54, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Mrrfrobj0x279279::set_p_mrr_fftargrtdetection(uint8_t* data,
    Mrr_frobj_0x279_279::Mrr_fftargrtdetectionType mrr_fftargrtdetection) {
  int x = mrr_fftargrtdetection;

  Byte to_set(data + 6);
  to_set.set_value(x, 6, 1);
}


Mrrfrobj0x279279* Mrrfrobj0x279279::set_mrr_peddetection(
    Mrr_frobj_0x279_279::Mrr_peddetectionType mrr_peddetection) {
  mrr_peddetection_ = mrr_peddetection;
  return this;
 }

// config detail: {'name': 'MRR_PedDetection', 'enum': {0: 'MRR_PEDDETECTION_NOT_DECTECTED', 1: 'MRR_PEDDETECTION_DECTECTED'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 55, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Mrrfrobj0x279279::set_p_mrr_peddetection(uint8_t* data,
    Mrr_frobj_0x279_279::Mrr_peddetectionType mrr_peddetection) {
  int x = mrr_peddetection;

  Byte to_set(data + 6);
  to_set.set_value(x, 7, 1);
}


Mrrfrobj0x279279* Mrrfrobj0x279279::set_mrr_f_object_dx(
    double mrr_f_object_dx) {
  mrr_f_object_dx_ = mrr_f_object_dx;
  return this;
 }

// config detail: {'name': 'MRR_F_Object_dx', 'enum': {4095: 'MRR_F_OBJECT_DX_NO_OBJECT'}, 'precision': 0.0625, 'len': 12, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|255.875]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Mrrfrobj0x279279::set_p_mrr_f_object_dx(uint8_t* data,
    double mrr_f_object_dx) {
  int x = mrr_f_object_dx / 0.062500;
  uint8_t t = 0;

  t = x & 0xF;
  Byte to_set0(data + 1);
  to_set0.set_value(t, 4, 4);
  x >>= 4;

  t = x & 0xFF;
  Byte to_set1(data + 0);
  to_set1.set_value(t, 0, 8);
}

}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto
