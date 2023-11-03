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

#include "modules/canbus/vehicle/cx75/protocol/pam_0x272_272.h"

#include "modules/drivers/canbus/common/byte.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

using ::jmc_auto::drivers::canbus::Byte;

const int32_t Pam0x272272::ID = 0x272;

// public
Pam0x272272::Pam0x272272() { Reset(); }

uint32_t Pam0x272272::GetPeriod() const {
  // TODO modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Pam0x272272::UpdateData(uint8_t* data) {
  set_p_apa_parkslot_state(data, apa_parkslot_state_);
  set_p_sdg_objectsts_frs(data, sdg_objectsts_frs_);
  set_p_sdg_objectstsrls(data, sdg_objectstsrls_);
  set_p_apa_selectedside(data, apa_selectedside_);
  set_p_sdg_objectstsrrs(data, sdg_objectstsrrs_);
  set_p_sdg_objectstsfrs_rrs(data, sdg_objectstsfrs_rrs_);
  set_p_apa_quit_indication(data, apa_quit_indication_);
  set_p_apa_recover_int_indication(data, apa_recover_int_indication_);
  set_p_sdg_objectstsfls_rls(data, sdg_objectstsfls_rls_);
  set_p_apa_indication(data, apa_indication_);
  set_p_sdg_objectstsfls(data, sdg_objectstsfls_);
}

void Pam0x272272::Reset() {
  // TODO you should check this manually
  apa_parkslot_state_ = Pam_0x272_272::APA_PARKSLOT_STATE_NO_SLOT;
  sdg_objectsts_frs_ = Pam_0x272_272::SDG_OBJECTSTS_FRS_NO_OBJECT;
  sdg_objectstsrls_ = Pam_0x272_272::SDG_OBJECTSTSRLS_NO_OBJECT;
  apa_selectedside_ = Pam_0x272_272::APA_SELECTEDSIDE_NONE;
  sdg_objectstsrrs_ = Pam_0x272_272::SDG_OBJECTSTSRRS_NO_OBJECT;
  sdg_objectstsfrs_rrs_ = Pam_0x272_272::SDG_OBJECTSTSFRS_RRS_NO_OBJECT;
  apa_quit_indication_ = Pam_0x272_272::APA_QUIT_INDICATION_NO_REQUEST;
  apa_recover_int_indication_ = Pam_0x272_272::APA_RECOVER_INT_INDICATION_NO_REQUEST;
  sdg_objectstsfls_rls_ = Pam_0x272_272::SDG_OBJECTSTSFLS_RLS_NO_OBJECT;
  apa_indication_ = Pam_0x272_272::APA_INDICATION_NO_ACTIVE;
  sdg_objectstsfls_ = Pam_0x272_272::SDG_OBJECTSTSFLS_NO_OBJECT;
}

Pam0x272272* Pam0x272272::set_apa_parkslot_state(
    Pam_0x272_272::Apa_parkslot_stateType apa_parkslot_state) {
  apa_parkslot_state_ = apa_parkslot_state;
  return this;
 }

// config detail: {'name': 'APA_Parkslot_State', 'enum': {0: 'APA_PARKSLOT_STATE_NO_SLOT', 1: 'APA_PARKSLOT_STATE_SLOT_LEFT', 2: 'APA_PARKSLOT_STATE_SLOT_RIGHT', 3: 'APA_PARKSLOT_STATE_INVALID'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 1, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Pam0x272272::set_p_apa_parkslot_state(uint8_t* data,
    Pam_0x272_272::Apa_parkslot_stateType apa_parkslot_state) {
  int x = apa_parkslot_state;

  Byte to_set(data + 0);
  to_set.set_value(x, 0, 2);
}


Pam0x272272* Pam0x272272::set_sdg_objectsts_frs(
    Pam_0x272_272::Sdg_objectsts_frsType sdg_objectsts_frs) {
  sdg_objectsts_frs_ = sdg_objectsts_frs;
  return this;
 }

// config detail: {'name': 'SDG_ObjectSts_FRS', 'enum': {0: 'SDG_OBJECTSTS_FRS_NO_OBJECT', 1: 'SDG_OBJECTSTS_FRS_OBJECT_IN_ZONE_1', 2: 'SDG_OBJECTSTS_FRS_OBJECT_IN_ZONE_2', 3: 'SDG_OBJECTSTS_FRS_OBJECT_IN_ZONE_3'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 12, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Pam0x272272::set_p_sdg_objectsts_frs(uint8_t* data,
    Pam_0x272_272::Sdg_objectsts_frsType sdg_objectsts_frs) {
  int x = sdg_objectsts_frs;

  Byte to_set(data + 1);
  to_set.set_value(x, 2, 3);
}


Pam0x272272* Pam0x272272::set_sdg_objectstsrls(
    Pam_0x272_272::Sdg_objectstsrlsType sdg_objectstsrls) {
  sdg_objectstsrls_ = sdg_objectstsrls;
  return this;
 }

// config detail: {'name': 'SDG_ObjectStsRLS', 'enum': {0: 'SDG_OBJECTSTSRLS_NO_OBJECT', 1: 'SDG_OBJECTSTSRLS_OBJECT_IN_ZONE_1', 2: 'SDG_OBJECTSTSRLS_OBJECT_IN_ZONE_2', 3: 'SDG_OBJECTSTSRLS_OBJECT_IN_ZONE_3'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 15, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Pam0x272272::set_p_sdg_objectstsrls(uint8_t* data,
    Pam_0x272_272::Sdg_objectstsrlsType sdg_objectstsrls) {
  int x = sdg_objectstsrls;

  Byte to_set(data + 1);
  to_set.set_value(x, 5, 3);
}


Pam0x272272* Pam0x272272::set_apa_selectedside(
    Pam_0x272_272::Apa_selectedsideType apa_selectedside) {
  apa_selectedside_ = apa_selectedside;
  return this;
 }

// config detail: {'name': 'APA_SelectedSide', 'enum': {0: 'APA_SELECTEDSIDE_NONE', 1: 'APA_SELECTEDSIDE_LEFT', 2: 'APA_SELECTEDSIDE_RIGHT', 3: 'APA_SELECTEDSIDE_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 17, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Pam0x272272::set_p_apa_selectedside(uint8_t* data,
    Pam_0x272_272::Apa_selectedsideType apa_selectedside) {
  int x = apa_selectedside;

  Byte to_set(data + 2);
  to_set.set_value(x, 0, 2);
}


Pam0x272272* Pam0x272272::set_sdg_objectstsrrs(
    Pam_0x272_272::Sdg_objectstsrrsType sdg_objectstsrrs) {
  sdg_objectstsrrs_ = sdg_objectstsrrs;
  return this;
 }

// config detail: {'name': 'SDG_ObjectStsRRS', 'enum': {0: 'SDG_OBJECTSTSRRS_NO_OBJECT', 1: 'SDG_OBJECTSTSRRS_OBJECT_IN_ZONE_1', 2: 'SDG_OBJECTSTSRRS_OBJECT_IN_ZONE_2', 3: 'SDG_OBJECTSTSRRS_OBJECT_IN_ZONE_3'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 20, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Pam0x272272::set_p_sdg_objectstsrrs(uint8_t* data,
    Pam_0x272_272::Sdg_objectstsrrsType sdg_objectstsrrs) {
  int x = sdg_objectstsrrs;

  Byte to_set(data + 2);
  to_set.set_value(x, 2, 3);
}


Pam0x272272* Pam0x272272::set_sdg_objectstsfrs_rrs(
    Pam_0x272_272::Sdg_objectstsfrs_rrsType sdg_objectstsfrs_rrs) {
  sdg_objectstsfrs_rrs_ = sdg_objectstsfrs_rrs;
  return this;
 }

// config detail: {'name': 'SDG_ObjectStsFRS_RRS', 'enum': {0: 'SDG_OBJECTSTSFRS_RRS_NO_OBJECT', 1: 'SDG_OBJECTSTSFRS_RRS_OBJECT_IN_ZONE_1', 2: 'SDG_OBJECTSTSFRS_RRS_OBJECT_IN_ZONE_2', 3: 'SDG_OBJECTSTSFRS_RRS_OBJECT_IN_ZONE_3'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 23, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Pam0x272272::set_p_sdg_objectstsfrs_rrs(uint8_t* data,
    Pam_0x272_272::Sdg_objectstsfrs_rrsType sdg_objectstsfrs_rrs) {
  int x = sdg_objectstsfrs_rrs;

  Byte to_set(data + 2);
  to_set.set_value(x, 5, 3);
}


Pam0x272272* Pam0x272272::set_apa_quit_indication(
    Pam_0x272_272::Apa_quit_indicationType apa_quit_indication) {
  apa_quit_indication_ = apa_quit_indication;
  return this;
 }

// config detail: {'name': 'APA_Quit_Indication', 'enum': {0: 'APA_QUIT_INDICATION_NO_REQUEST', 1: 'APA_QUIT_INDICATION_REQ_QUIT_EXTERNAL_ECU_FAILURE', 2: 'APA_QUIT_INDICATION_REQ_QUIT_APA_FAILURE', 3: 'APA_QUIT_INDICATION_REQ_QUIT_SPEED_HIGH', 4: 'APA_QUIT_INDICATION_REQ_QUIT_TIMING_OVERFLOW', 5: 'APA_QUIT_INDICATION_REQ_QUIT_MOVE_TIMES_OVERFLOW', 6: 'APA_QUIT_INDICATION_REQ_QUIT_RECOVER_INT_TIMS_OVERFLOW', 7: 'APA_QUIT_INDICATION_REQ_QUIT_GAS_PEDAL', 8: 'APA_QUIT_INDICATION_REQ_QUIT_GEAR_INTERVENTION', 9: 'APA_QUIT_INDICATION_REQ_QUIT_EXTERNAL_ECU_ACTVIE', 10: 'APA_QUIT_INDICATION_REQ_QUIT_ACC', 11: 'APA_QUIT_INDICATION_REQ_QUIT_TRAJECTORY', 12: 'APA_QUIT_INDICATION_REQ_QUIT_SPACE_LIMIT', 13: 'APA_QUIT_INDICATION_REQ_QUIT_NO_OBJECT_IN_FRONT', 14: 'APA_QUIT_INDICATION_REQ_QUIT_SEAT_BELT_UNBUCKLE', 15: 'APA_QUIT_INDICATION_REQ_QUIT_DRIVER_DOOR', 16: 'APA_QUIT_INDICATION_REQ_QUIT_EPB_APPLY', 17: 'APA_QUIT_INDICATION_REQ_QUIT_VEHICLE_BLOCK', 18: 'APA_QUIT_INDICATION_REQ_QUIT_GEAR_INTERRUPT', 19: 'APA_QUIT_INDICATION_REQ_QUIT_STEERING_WHEEL_HANDS_ON', 20: 'APA_QUIT_INDICATION_REQ_QUIT_ABOVE_SLOPE_LIMT', 21: 'APA_QUIT_INDICATION_RESERVED', 22: 'APA_QUIT_INDICATION_RESERVED', 23: 'APA_QUIT_INDICATION_RESERVED', 24: 'APA_QUIT_INDICATION_RESERVED', 25: 'APA_QUIT_INDICATION_RESERVED', 26: 'APA_QUIT_INDICATION_RESERVED', 27: 'APA_QUIT_INDICATION_RESERVED', 28: 'APA_QUIT_INDICATION_RESERVED', 29: 'APA_QUIT_INDICATION_RESERVED', 30: 'APA_QUIT_INDICATION_RESERVED', 31: 'APA_QUIT_INDICATION_RESERVED'}, 'precision': 1.0, 'len': 5, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|31]', 'bit': 36, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Pam0x272272::set_p_apa_quit_indication(uint8_t* data,
    Pam_0x272_272::Apa_quit_indicationType apa_quit_indication) {
  int x = apa_quit_indication;

  Byte to_set(data + 4);
  to_set.set_value(x, 0, 5);
}


Pam0x272272* Pam0x272272::set_apa_recover_int_indication(
    Pam_0x272_272::Apa_recover_int_indicationType apa_recover_int_indication) {
  apa_recover_int_indication_ = apa_recover_int_indication;
  return this;
 }

// config detail: {'name': 'APA_Recover_INT_Indication', 'enum': {0: 'APA_RECOVER_INT_INDICATION_NO_REQUEST', 1: 'APA_RECOVER_INT_INDICATION_REQ_RECOVER_DEAD_MAN_SWITCH_RELEASE', 2: 'APA_RECOVER_INT_INDICATION_REQ_RECOVER_FOUND_OBSTACLE_IN_TRAJECTORY', 3: 'APA_RECOVER_INT_INDICATION_REQ_RECOVER_NON_DRIVER_DOOR_OPEN', 4: 'APA_RECOVER_INT_INDICATION_REQ_RECOVER_BRAKE_PEDAL_RELEASE', 5: 'APA_RECOVER_INT_INDICATION_RESERVED', 6: 'APA_RECOVER_INT_INDICATION_RESERVED', 7: 'APA_RECOVER_INT_INDICATION_RESERVED'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 39, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Pam0x272272::set_p_apa_recover_int_indication(uint8_t* data,
    Pam_0x272_272::Apa_recover_int_indicationType apa_recover_int_indication) {
  int x = apa_recover_int_indication;

  Byte to_set(data + 4);
  to_set.set_value(x, 5, 3);
}


Pam0x272272* Pam0x272272::set_sdg_objectstsfls_rls(
    Pam_0x272_272::Sdg_objectstsfls_rlsType sdg_objectstsfls_rls) {
  sdg_objectstsfls_rls_ = sdg_objectstsfls_rls;
  return this;
 }

// config detail: {'name': 'SDG_ObjectStsFLS_RLS', 'enum': {0: 'SDG_OBJECTSTSFLS_RLS_NO_OBJECT', 1: 'SDG_OBJECTSTSFLS_RLS_OBJECT_IN_ZONE_1', 2: 'SDG_OBJECTSTSFLS_RLS_OBJECT_IN_ZONE_2', 3: 'SDG_OBJECTSTSFLS_RLS_OBJECT_IN_ZONE_3'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 4, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Pam0x272272::set_p_sdg_objectstsfls_rls(uint8_t* data,
    Pam_0x272_272::Sdg_objectstsfls_rlsType sdg_objectstsfls_rls) {
  int x = sdg_objectstsfls_rls;

  Byte to_set(data + 0);
  to_set.set_value(x, 2, 3);
}


Pam0x272272* Pam0x272272::set_apa_indication(
    Pam_0x272_272::Apa_indicationType apa_indication) {
  apa_indication_ = apa_indication;
  return this;
 }

// config detail: {'name': 'APA_Indication', 'enum': {0: 'APA_INDICATION_NO_ACTIVE', 1: 'APA_INDICATION_PPSC_ACTIVE', 2: 'APA_INDICATION_CPSC_ACTIVE', 3: 'APA_INDICATION_POC_ACTIVE', 4: 'APA_INDICATION_RESERVED', 5: 'APA_INDICATION_RESERVED', 6: 'APA_INDICATION_RESERVED', 7: 'APA_INDICATION_RESERVED'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 47, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Pam0x272272::set_p_apa_indication(uint8_t* data,
    Pam_0x272_272::Apa_indicationType apa_indication) {
  int x = apa_indication;

  Byte to_set(data + 5);
  to_set.set_value(x, 5, 3);
}


Pam0x272272* Pam0x272272::set_sdg_objectstsfls(
    Pam_0x272_272::Sdg_objectstsflsType sdg_objectstsfls) {
  sdg_objectstsfls_ = sdg_objectstsfls;
  return this;
 }

// config detail: {'name': 'SDG_ObjectStsFLS', 'enum': {0: 'SDG_OBJECTSTSFLS_NO_OBJECT', 1: 'SDG_OBJECTSTSFLS_OBJECT_IN_ZONE_1', 2: 'SDG_OBJECTSTSFLS_OBJECT_IN_ZONE_2', 3: 'SDG_OBJECTSTSFLS_OBJECT_IN_ZONE_3'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Pam0x272272::set_p_sdg_objectstsfls(uint8_t* data,
    Pam_0x272_272::Sdg_objectstsflsType sdg_objectstsfls) {
  int x = sdg_objectstsfls;

  Byte to_set(data + 0);
  to_set.set_value(x, 5, 3);
}

}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto
