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

#ifndef MODULES_CANBUS_VEHICLE_cx75_PROTOCOL_PAM_0X272_272_H_
#define MODULES_CANBUS_VEHICLE_cx75_PROTOCOL_PAM_0X272_272_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

class Pam0x272272 : public ::jmc_auto::drivers::canbus::ProtocolData<
                    ::jmc_auto::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Pam0x272272();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'name': 'APA_Parkslot_State', 'enum': {0: 'APA_PARKSLOT_STATE_NO_SLOT', 1: 'APA_PARKSLOT_STATE_SLOT_LEFT', 2: 'APA_PARKSLOT_STATE_SLOT_RIGHT', 3: 'APA_PARKSLOT_STATE_INVALID'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 1, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Pam0x272272* set_apa_parkslot_state(Pam_0x272_272::Apa_parkslot_stateType apa_parkslot_state);

  // config detail: {'name': 'SDG_ObjectSts_FRS', 'enum': {0: 'SDG_OBJECTSTS_FRS_NO_OBJECT', 1: 'SDG_OBJECTSTS_FRS_OBJECT_IN_ZONE_1', 2: 'SDG_OBJECTSTS_FRS_OBJECT_IN_ZONE_2', 3: 'SDG_OBJECTSTS_FRS_OBJECT_IN_ZONE_3'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 12, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Pam0x272272* set_sdg_objectsts_frs(Pam_0x272_272::Sdg_objectsts_frsType sdg_objectsts_frs);

  // config detail: {'name': 'SDG_ObjectStsRLS', 'enum': {0: 'SDG_OBJECTSTSRLS_NO_OBJECT', 1: 'SDG_OBJECTSTSRLS_OBJECT_IN_ZONE_1', 2: 'SDG_OBJECTSTSRLS_OBJECT_IN_ZONE_2', 3: 'SDG_OBJECTSTSRLS_OBJECT_IN_ZONE_3'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 15, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Pam0x272272* set_sdg_objectstsrls(Pam_0x272_272::Sdg_objectstsrlsType sdg_objectstsrls);

  // config detail: {'name': 'APA_SelectedSide', 'enum': {0: 'APA_SELECTEDSIDE_NONE', 1: 'APA_SELECTEDSIDE_LEFT', 2: 'APA_SELECTEDSIDE_RIGHT', 3: 'APA_SELECTEDSIDE_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 17, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Pam0x272272* set_apa_selectedside(Pam_0x272_272::Apa_selectedsideType apa_selectedside);

  // config detail: {'name': 'SDG_ObjectStsRRS', 'enum': {0: 'SDG_OBJECTSTSRRS_NO_OBJECT', 1: 'SDG_OBJECTSTSRRS_OBJECT_IN_ZONE_1', 2: 'SDG_OBJECTSTSRRS_OBJECT_IN_ZONE_2', 3: 'SDG_OBJECTSTSRRS_OBJECT_IN_ZONE_3'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 20, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Pam0x272272* set_sdg_objectstsrrs(Pam_0x272_272::Sdg_objectstsrrsType sdg_objectstsrrs);

  // config detail: {'name': 'SDG_ObjectStsFRS_RRS', 'enum': {0: 'SDG_OBJECTSTSFRS_RRS_NO_OBJECT', 1: 'SDG_OBJECTSTSFRS_RRS_OBJECT_IN_ZONE_1', 2: 'SDG_OBJECTSTSFRS_RRS_OBJECT_IN_ZONE_2', 3: 'SDG_OBJECTSTSFRS_RRS_OBJECT_IN_ZONE_3'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 23, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Pam0x272272* set_sdg_objectstsfrs_rrs(Pam_0x272_272::Sdg_objectstsfrs_rrsType sdg_objectstsfrs_rrs);

  // config detail: {'name': 'APA_Quit_Indication', 'enum': {0: 'APA_QUIT_INDICATION_NO_REQUEST', 1: 'APA_QUIT_INDICATION_REQ_QUIT_EXTERNAL_ECU_FAILURE', 2: 'APA_QUIT_INDICATION_REQ_QUIT_APA_FAILURE', 3: 'APA_QUIT_INDICATION_REQ_QUIT_SPEED_HIGH', 4: 'APA_QUIT_INDICATION_REQ_QUIT_TIMING_OVERFLOW', 5: 'APA_QUIT_INDICATION_REQ_QUIT_MOVE_TIMES_OVERFLOW', 6: 'APA_QUIT_INDICATION_REQ_QUIT_RECOVER_INT_TIMS_OVERFLOW', 7: 'APA_QUIT_INDICATION_REQ_QUIT_GAS_PEDAL', 8: 'APA_QUIT_INDICATION_REQ_QUIT_GEAR_INTERVENTION', 9: 'APA_QUIT_INDICATION_REQ_QUIT_EXTERNAL_ECU_ACTVIE', 10: 'APA_QUIT_INDICATION_REQ_QUIT_ACC', 11: 'APA_QUIT_INDICATION_REQ_QUIT_TRAJECTORY', 12: 'APA_QUIT_INDICATION_REQ_QUIT_SPACE_LIMIT', 13: 'APA_QUIT_INDICATION_REQ_QUIT_NO_OBJECT_IN_FRONT', 14: 'APA_QUIT_INDICATION_REQ_QUIT_SEAT_BELT_UNBUCKLE', 15: 'APA_QUIT_INDICATION_REQ_QUIT_DRIVER_DOOR', 16: 'APA_QUIT_INDICATION_REQ_QUIT_EPB_APPLY', 17: 'APA_QUIT_INDICATION_REQ_QUIT_VEHICLE_BLOCK', 18: 'APA_QUIT_INDICATION_REQ_QUIT_GEAR_INTERRUPT', 19: 'APA_QUIT_INDICATION_REQ_QUIT_STEERING_WHEEL_HANDS_ON', 20: 'APA_QUIT_INDICATION_REQ_QUIT_ABOVE_SLOPE_LIMT', 21: 'APA_QUIT_INDICATION_RESERVED', 22: 'APA_QUIT_INDICATION_RESERVED', 23: 'APA_QUIT_INDICATION_RESERVED', 24: 'APA_QUIT_INDICATION_RESERVED', 25: 'APA_QUIT_INDICATION_RESERVED', 26: 'APA_QUIT_INDICATION_RESERVED', 27: 'APA_QUIT_INDICATION_RESERVED', 28: 'APA_QUIT_INDICATION_RESERVED', 29: 'APA_QUIT_INDICATION_RESERVED', 30: 'APA_QUIT_INDICATION_RESERVED', 31: 'APA_QUIT_INDICATION_RESERVED'}, 'precision': 1.0, 'len': 5, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|31]', 'bit': 36, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Pam0x272272* set_apa_quit_indication(Pam_0x272_272::Apa_quit_indicationType apa_quit_indication);

  // config detail: {'name': 'APA_Recover_INT_Indication', 'enum': {0: 'APA_RECOVER_INT_INDICATION_NO_REQUEST', 1: 'APA_RECOVER_INT_INDICATION_REQ_RECOVER_DEAD_MAN_SWITCH_RELEASE', 2: 'APA_RECOVER_INT_INDICATION_REQ_RECOVER_FOUND_OBSTACLE_IN_TRAJECTORY', 3: 'APA_RECOVER_INT_INDICATION_REQ_RECOVER_NON_DRIVER_DOOR_OPEN', 4: 'APA_RECOVER_INT_INDICATION_REQ_RECOVER_BRAKE_PEDAL_RELEASE', 5: 'APA_RECOVER_INT_INDICATION_RESERVED', 6: 'APA_RECOVER_INT_INDICATION_RESERVED', 7: 'APA_RECOVER_INT_INDICATION_RESERVED'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 39, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Pam0x272272* set_apa_recover_int_indication(Pam_0x272_272::Apa_recover_int_indicationType apa_recover_int_indication);

  // config detail: {'name': 'SDG_ObjectStsFLS_RLS', 'enum': {0: 'SDG_OBJECTSTSFLS_RLS_NO_OBJECT', 1: 'SDG_OBJECTSTSFLS_RLS_OBJECT_IN_ZONE_1', 2: 'SDG_OBJECTSTSFLS_RLS_OBJECT_IN_ZONE_2', 3: 'SDG_OBJECTSTSFLS_RLS_OBJECT_IN_ZONE_3'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 4, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Pam0x272272* set_sdg_objectstsfls_rls(Pam_0x272_272::Sdg_objectstsfls_rlsType sdg_objectstsfls_rls);

  // config detail: {'name': 'APA_Indication', 'enum': {0: 'APA_INDICATION_NO_ACTIVE', 1: 'APA_INDICATION_PPSC_ACTIVE', 2: 'APA_INDICATION_CPSC_ACTIVE', 3: 'APA_INDICATION_POC_ACTIVE', 4: 'APA_INDICATION_RESERVED', 5: 'APA_INDICATION_RESERVED', 6: 'APA_INDICATION_RESERVED', 7: 'APA_INDICATION_RESERVED'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 47, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Pam0x272272* set_apa_indication(Pam_0x272_272::Apa_indicationType apa_indication);

  // config detail: {'name': 'SDG_ObjectStsFLS', 'enum': {0: 'SDG_OBJECTSTSFLS_NO_OBJECT', 1: 'SDG_OBJECTSTSFLS_OBJECT_IN_ZONE_1', 2: 'SDG_OBJECTSTSFLS_OBJECT_IN_ZONE_2', 3: 'SDG_OBJECTSTSFLS_OBJECT_IN_ZONE_3'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Pam0x272272* set_sdg_objectstsfls(Pam_0x272_272::Sdg_objectstsflsType sdg_objectstsfls);

 private:

  // config detail: {'name': 'APA_Parkslot_State', 'enum': {0: 'APA_PARKSLOT_STATE_NO_SLOT', 1: 'APA_PARKSLOT_STATE_SLOT_LEFT', 2: 'APA_PARKSLOT_STATE_SLOT_RIGHT', 3: 'APA_PARKSLOT_STATE_INVALID'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 1, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_apa_parkslot_state(uint8_t* data, Pam_0x272_272::Apa_parkslot_stateType apa_parkslot_state);

  // config detail: {'name': 'SDG_ObjectSts_FRS', 'enum': {0: 'SDG_OBJECTSTS_FRS_NO_OBJECT', 1: 'SDG_OBJECTSTS_FRS_OBJECT_IN_ZONE_1', 2: 'SDG_OBJECTSTS_FRS_OBJECT_IN_ZONE_2', 3: 'SDG_OBJECTSTS_FRS_OBJECT_IN_ZONE_3'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 12, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_sdg_objectsts_frs(uint8_t* data, Pam_0x272_272::Sdg_objectsts_frsType sdg_objectsts_frs);

  // config detail: {'name': 'SDG_ObjectStsRLS', 'enum': {0: 'SDG_OBJECTSTSRLS_NO_OBJECT', 1: 'SDG_OBJECTSTSRLS_OBJECT_IN_ZONE_1', 2: 'SDG_OBJECTSTSRLS_OBJECT_IN_ZONE_2', 3: 'SDG_OBJECTSTSRLS_OBJECT_IN_ZONE_3'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 15, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_sdg_objectstsrls(uint8_t* data, Pam_0x272_272::Sdg_objectstsrlsType sdg_objectstsrls);

  // config detail: {'name': 'APA_SelectedSide', 'enum': {0: 'APA_SELECTEDSIDE_NONE', 1: 'APA_SELECTEDSIDE_LEFT', 2: 'APA_SELECTEDSIDE_RIGHT', 3: 'APA_SELECTEDSIDE_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 17, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_apa_selectedside(uint8_t* data, Pam_0x272_272::Apa_selectedsideType apa_selectedside);

  // config detail: {'name': 'SDG_ObjectStsRRS', 'enum': {0: 'SDG_OBJECTSTSRRS_NO_OBJECT', 1: 'SDG_OBJECTSTSRRS_OBJECT_IN_ZONE_1', 2: 'SDG_OBJECTSTSRRS_OBJECT_IN_ZONE_2', 3: 'SDG_OBJECTSTSRRS_OBJECT_IN_ZONE_3'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 20, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_sdg_objectstsrrs(uint8_t* data, Pam_0x272_272::Sdg_objectstsrrsType sdg_objectstsrrs);

  // config detail: {'name': 'SDG_ObjectStsFRS_RRS', 'enum': {0: 'SDG_OBJECTSTSFRS_RRS_NO_OBJECT', 1: 'SDG_OBJECTSTSFRS_RRS_OBJECT_IN_ZONE_1', 2: 'SDG_OBJECTSTSFRS_RRS_OBJECT_IN_ZONE_2', 3: 'SDG_OBJECTSTSFRS_RRS_OBJECT_IN_ZONE_3'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 23, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_sdg_objectstsfrs_rrs(uint8_t* data, Pam_0x272_272::Sdg_objectstsfrs_rrsType sdg_objectstsfrs_rrs);

  // config detail: {'name': 'APA_Quit_Indication', 'enum': {0: 'APA_QUIT_INDICATION_NO_REQUEST', 1: 'APA_QUIT_INDICATION_REQ_QUIT_EXTERNAL_ECU_FAILURE', 2: 'APA_QUIT_INDICATION_REQ_QUIT_APA_FAILURE', 3: 'APA_QUIT_INDICATION_REQ_QUIT_SPEED_HIGH', 4: 'APA_QUIT_INDICATION_REQ_QUIT_TIMING_OVERFLOW', 5: 'APA_QUIT_INDICATION_REQ_QUIT_MOVE_TIMES_OVERFLOW', 6: 'APA_QUIT_INDICATION_REQ_QUIT_RECOVER_INT_TIMS_OVERFLOW', 7: 'APA_QUIT_INDICATION_REQ_QUIT_GAS_PEDAL', 8: 'APA_QUIT_INDICATION_REQ_QUIT_GEAR_INTERVENTION', 9: 'APA_QUIT_INDICATION_REQ_QUIT_EXTERNAL_ECU_ACTVIE', 10: 'APA_QUIT_INDICATION_REQ_QUIT_ACC', 11: 'APA_QUIT_INDICATION_REQ_QUIT_TRAJECTORY', 12: 'APA_QUIT_INDICATION_REQ_QUIT_SPACE_LIMIT', 13: 'APA_QUIT_INDICATION_REQ_QUIT_NO_OBJECT_IN_FRONT', 14: 'APA_QUIT_INDICATION_REQ_QUIT_SEAT_BELT_UNBUCKLE', 15: 'APA_QUIT_INDICATION_REQ_QUIT_DRIVER_DOOR', 16: 'APA_QUIT_INDICATION_REQ_QUIT_EPB_APPLY', 17: 'APA_QUIT_INDICATION_REQ_QUIT_VEHICLE_BLOCK', 18: 'APA_QUIT_INDICATION_REQ_QUIT_GEAR_INTERRUPT', 19: 'APA_QUIT_INDICATION_REQ_QUIT_STEERING_WHEEL_HANDS_ON', 20: 'APA_QUIT_INDICATION_REQ_QUIT_ABOVE_SLOPE_LIMT', 21: 'APA_QUIT_INDICATION_RESERVED', 22: 'APA_QUIT_INDICATION_RESERVED', 23: 'APA_QUIT_INDICATION_RESERVED', 24: 'APA_QUIT_INDICATION_RESERVED', 25: 'APA_QUIT_INDICATION_RESERVED', 26: 'APA_QUIT_INDICATION_RESERVED', 27: 'APA_QUIT_INDICATION_RESERVED', 28: 'APA_QUIT_INDICATION_RESERVED', 29: 'APA_QUIT_INDICATION_RESERVED', 30: 'APA_QUIT_INDICATION_RESERVED', 31: 'APA_QUIT_INDICATION_RESERVED'}, 'precision': 1.0, 'len': 5, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|31]', 'bit': 36, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_apa_quit_indication(uint8_t* data, Pam_0x272_272::Apa_quit_indicationType apa_quit_indication);

  // config detail: {'name': 'APA_Recover_INT_Indication', 'enum': {0: 'APA_RECOVER_INT_INDICATION_NO_REQUEST', 1: 'APA_RECOVER_INT_INDICATION_REQ_RECOVER_DEAD_MAN_SWITCH_RELEASE', 2: 'APA_RECOVER_INT_INDICATION_REQ_RECOVER_FOUND_OBSTACLE_IN_TRAJECTORY', 3: 'APA_RECOVER_INT_INDICATION_REQ_RECOVER_NON_DRIVER_DOOR_OPEN', 4: 'APA_RECOVER_INT_INDICATION_REQ_RECOVER_BRAKE_PEDAL_RELEASE', 5: 'APA_RECOVER_INT_INDICATION_RESERVED', 6: 'APA_RECOVER_INT_INDICATION_RESERVED', 7: 'APA_RECOVER_INT_INDICATION_RESERVED'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 39, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_apa_recover_int_indication(uint8_t* data, Pam_0x272_272::Apa_recover_int_indicationType apa_recover_int_indication);

  // config detail: {'name': 'SDG_ObjectStsFLS_RLS', 'enum': {0: 'SDG_OBJECTSTSFLS_RLS_NO_OBJECT', 1: 'SDG_OBJECTSTSFLS_RLS_OBJECT_IN_ZONE_1', 2: 'SDG_OBJECTSTSFLS_RLS_OBJECT_IN_ZONE_2', 3: 'SDG_OBJECTSTSFLS_RLS_OBJECT_IN_ZONE_3'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 4, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_sdg_objectstsfls_rls(uint8_t* data, Pam_0x272_272::Sdg_objectstsfls_rlsType sdg_objectstsfls_rls);

  // config detail: {'name': 'APA_Indication', 'enum': {0: 'APA_INDICATION_NO_ACTIVE', 1: 'APA_INDICATION_PPSC_ACTIVE', 2: 'APA_INDICATION_CPSC_ACTIVE', 3: 'APA_INDICATION_POC_ACTIVE', 4: 'APA_INDICATION_RESERVED', 5: 'APA_INDICATION_RESERVED', 6: 'APA_INDICATION_RESERVED', 7: 'APA_INDICATION_RESERVED'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 47, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_apa_indication(uint8_t* data, Pam_0x272_272::Apa_indicationType apa_indication);

  // config detail: {'name': 'SDG_ObjectStsFLS', 'enum': {0: 'SDG_OBJECTSTSFLS_NO_OBJECT', 1: 'SDG_OBJECTSTSFLS_OBJECT_IN_ZONE_1', 2: 'SDG_OBJECTSTSFLS_OBJECT_IN_ZONE_2', 3: 'SDG_OBJECTSTSFLS_OBJECT_IN_ZONE_3'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_sdg_objectstsfls(uint8_t* data, Pam_0x272_272::Sdg_objectstsflsType sdg_objectstsfls);

 private:
  Pam_0x272_272::Apa_parkslot_stateType apa_parkslot_state_;
  Pam_0x272_272::Sdg_objectsts_frsType sdg_objectsts_frs_;
  Pam_0x272_272::Sdg_objectstsrlsType sdg_objectstsrls_;
  Pam_0x272_272::Apa_selectedsideType apa_selectedside_;
  Pam_0x272_272::Sdg_objectstsrrsType sdg_objectstsrrs_;
  Pam_0x272_272::Sdg_objectstsfrs_rrsType sdg_objectstsfrs_rrs_;
  Pam_0x272_272::Apa_quit_indicationType apa_quit_indication_;
  Pam_0x272_272::Apa_recover_int_indicationType apa_recover_int_indication_;
  Pam_0x272_272::Sdg_objectstsfls_rlsType sdg_objectstsfls_rls_;
  Pam_0x272_272::Apa_indicationType apa_indication_;
  Pam_0x272_272::Sdg_objectstsflsType sdg_objectstsfls_;
};

}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_cx75_PROTOCOL_PAM_0X272_272_H_
