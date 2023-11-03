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

#ifndef MODULES_CANBUS_VEHICLE_cx75_PROTOCOL_MRR_0X239_239_H_
#define MODULES_CANBUS_VEHICLE_cx75_PROTOCOL_MRR_0X239_239_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

class Mrr0x239239 : public ::jmc_auto::drivers::canbus::ProtocolData<
                    ::jmc_auto::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Mrr0x239239();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;
  void checksum_rolling();


  // config detail: {'name': 'ACC_UpperComftBandReq', 'enum': {0: 'ACC_UPPERCOMFTBANDREQ_NO_DEMAND', 1: 'ACC_UPPERCOMFTBANDREQ_DEMAND'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 0, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Mrr0x239239* set_acc_uppercomftbandreq(Mrr_0x239_239::Acc_uppercomftbandreqType acc_uppercomftbandreq);

  // config detail: {'name': 'ACC_BrakePreferred', 'enum': {0: 'ACC_BRAKEPREFERRED_NO_DEMAND', 1: 'ACC_BRAKEPREFERRED_DEMAND'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 1, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Mrr0x239239* set_acc_brakepreferred(Mrr_0x239_239::Acc_brakepreferredType acc_brakepreferred);

  // config detail: {'name': 'EBA_Req', 'enum': {0: 'EBA_REQ_NO_DEMAND', 1: 'EBA_REQ_DEMAND'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 12, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Mrr0x239239* set_eba_req(Mrr_0x239_239::Eba_reqType eba_req);

  // config detail: {'name': 'AEB_Req', 'enum': {0: 'AEB_REQ_NO_DEMAND', 1: 'AEB_REQ_DEMAND'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 13, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Mrr0x239239* set_aeb_req(Mrr_0x239_239::Aeb_reqType aeb_req);

  // config detail: {'name': 'ACC_StandstillReq', 'enum': {0: 'ACC_STANDSTILLREQ_NO_DEMAND', 1: 'ACC_STANDSTILLREQ_DEMAND'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 14, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Mrr0x239239* set_acc_standstillreq(Mrr_0x239_239::Acc_standstillreqType acc_standstillreq);

  // config detail: {'name': 'ACC_DriveOff', 'enum': {0: 'ACC_DRIVEOFF_NO_DEMAND', 1: 'ACC_DRIVEOFF_DEMAND'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 15, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Mrr0x239239* set_acc_driveoff(Mrr_0x239_239::Acc_driveoffType acc_driveoff);

  // config detail: {'name': 'AWB_Level', 'enum': {0: 'AWB_LEVEL_NO_LEVEL', 1: 'AWB_LEVEL_LEVEL_1', 2: 'AWB_LEVEL_LEVEL_2', 3: 'AWB_LEVEL_LEVEL_3', 4: 'AWB_LEVEL_LEVEL_4'}, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|15]', 'bit': 19, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Mrr0x239239* set_awb_level(Mrr_0x239_239::Awb_levelType awb_level);

  // config detail: {'name': 'ABP_Req', 'enum': {0: 'ABP_REQ_NO_DEMAND', 1: 'ABP_REQ_DEMAND'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 21, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Mrr0x239239* set_abp_req(Mrr_0x239_239::Abp_reqType abp_req);

  // config detail: {'name': 'AWB_Req', 'enum': {0: 'AWB_REQ_NO_DEMAND', 1: 'AWB_REQ_DEMAND'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 22, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Mrr0x239239* set_awb_req(Mrr_0x239_239::Awb_reqType awb_req);

  // config detail: {'name': 'ABA_Req', 'enum': {0: 'ABA_REQ_NO_DEMAND', 1: 'ABA_REQ_DEMAND'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 23, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Mrr0x239239* set_aba_req(Mrr_0x239_239::Aba_reqType aba_req);

  // config detail: {'name': 'AEB_TgtAx', 'offset': -16.0, 'precision': 0.05, 'len': 16, 'is_signed_var': False, 'physical_range': '[-16|16]', 'bit': 31, 'type': 'double', 'order': 'motorola', 'physical_unit': 'm s2'}
  Mrr0x239239* set_aeb_tgtax(double aeb_tgtax);

  // config detail: {'name': 'ACC_State', 'enum': {0: 'ACC_STATE_OFF_MODE', 1: 'ACC_STATE_PASSIVE_MODE', 2: 'ACC_STATE_STAND_BY_MODE', 3: 'ACC_STATE_ACTIVE_CONTROL_MODE', 4: 'ACC_STATE_BRAKE_ONLY_MODE', 5: 'ACC_STATE_OVERRIDE', 6: 'ACC_STATE_STANDSTILL', 7: 'ACC_STATE_FAILURE_MODE'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 5, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Mrr0x239239* set_acc_state(Mrr_0x239_239::Acc_stateType acc_state);

  // config detail: {'name': 'Rolling_counter_0x239', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  Mrr0x239239* set_rolling_counter_0x239(int rolling_counter_0x239);

  // config detail: {'name': 'Checksum_0x239', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  Mrr0x239239* set_checksum_0x239(int checksum_0x239);

  // config detail: {'name': 'ShutdownMode', 'enum': {0: 'SHUTDOWNMODE_SOFT_OFF', 1: 'SHUTDOWNMODE_FAST_OFF', 2: 'SHUTDOWNMODE_RESERVED', 3: 'SHUTDOWNMODE_INITIAL'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Mrr0x239239* set_shutdownmode(Mrr_0x239_239::ShutdownmodeType shutdownmode);

  // config detail: {'name': 'ABA_Level', 'enum': {0: 'ABA_LEVEL_LEVEL_0', 1: 'ABA_LEVEL_LEVEL_1', 2: 'ABA_LEVEL_LEVEL_2', 3: 'ABA_LEVEL_LEVEL_3'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 9, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Mrr0x239239* set_aba_level(Mrr_0x239_239::Aba_levelType aba_level);

 private:

  // config detail: {'name': 'ACC_UpperComftBandReq', 'enum': {0: 'ACC_UPPERCOMFTBANDREQ_NO_DEMAND', 1: 'ACC_UPPERCOMFTBANDREQ_DEMAND'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 0, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_acc_uppercomftbandreq(uint8_t* data, Mrr_0x239_239::Acc_uppercomftbandreqType acc_uppercomftbandreq);

  // config detail: {'name': 'ACC_BrakePreferred', 'enum': {0: 'ACC_BRAKEPREFERRED_NO_DEMAND', 1: 'ACC_BRAKEPREFERRED_DEMAND'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 1, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_acc_brakepreferred(uint8_t* data, Mrr_0x239_239::Acc_brakepreferredType acc_brakepreferred);

  // config detail: {'name': 'EBA_Req', 'enum': {0: 'EBA_REQ_NO_DEMAND', 1: 'EBA_REQ_DEMAND'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 12, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_eba_req(uint8_t* data, Mrr_0x239_239::Eba_reqType eba_req);

  // config detail: {'name': 'AEB_Req', 'enum': {0: 'AEB_REQ_NO_DEMAND', 1: 'AEB_REQ_DEMAND'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 13, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_aeb_req(uint8_t* data, Mrr_0x239_239::Aeb_reqType aeb_req);

  // config detail: {'name': 'ACC_StandstillReq', 'enum': {0: 'ACC_STANDSTILLREQ_NO_DEMAND', 1: 'ACC_STANDSTILLREQ_DEMAND'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 14, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_acc_standstillreq(uint8_t* data, Mrr_0x239_239::Acc_standstillreqType acc_standstillreq);

  // config detail: {'name': 'ACC_DriveOff', 'enum': {0: 'ACC_DRIVEOFF_NO_DEMAND', 1: 'ACC_DRIVEOFF_DEMAND'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 15, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_acc_driveoff(uint8_t* data, Mrr_0x239_239::Acc_driveoffType acc_driveoff);

  // config detail: {'name': 'AWB_Level', 'enum': {0: 'AWB_LEVEL_NO_LEVEL', 1: 'AWB_LEVEL_LEVEL_1', 2: 'AWB_LEVEL_LEVEL_2', 3: 'AWB_LEVEL_LEVEL_3', 4: 'AWB_LEVEL_LEVEL_4'}, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|15]', 'bit': 19, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_awb_level(uint8_t* data, Mrr_0x239_239::Awb_levelType awb_level);

  // config detail: {'name': 'ABP_Req', 'enum': {0: 'ABP_REQ_NO_DEMAND', 1: 'ABP_REQ_DEMAND'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 21, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_abp_req(uint8_t* data, Mrr_0x239_239::Abp_reqType abp_req);

  // config detail: {'name': 'AWB_Req', 'enum': {0: 'AWB_REQ_NO_DEMAND', 1: 'AWB_REQ_DEMAND'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 22, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_awb_req(uint8_t* data, Mrr_0x239_239::Awb_reqType awb_req);

  // config detail: {'name': 'ABA_Req', 'enum': {0: 'ABA_REQ_NO_DEMAND', 1: 'ABA_REQ_DEMAND'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 23, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_aba_req(uint8_t* data, Mrr_0x239_239::Aba_reqType aba_req);

  // config detail: {'name': 'AEB_TgtAx', 'offset': -16.0, 'precision': 0.05, 'len': 16, 'is_signed_var': False, 'physical_range': '[-16|16]', 'bit': 31, 'type': 'double', 'order': 'motorola', 'physical_unit': 'm s2'}
  void set_p_aeb_tgtax(uint8_t* data, double aeb_tgtax);

  // config detail: {'name': 'ACC_State', 'enum': {0: 'ACC_STATE_OFF_MODE', 1: 'ACC_STATE_PASSIVE_MODE', 2: 'ACC_STATE_STAND_BY_MODE', 3: 'ACC_STATE_ACTIVE_CONTROL_MODE', 4: 'ACC_STATE_BRAKE_ONLY_MODE', 5: 'ACC_STATE_OVERRIDE', 6: 'ACC_STATE_STANDSTILL', 7: 'ACC_STATE_FAILURE_MODE'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 5, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_acc_state(uint8_t* data, Mrr_0x239_239::Acc_stateType acc_state);

  // config detail: {'name': 'Rolling_counter_0x239', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  void set_p_rolling_counter_0x239(uint8_t* data, int rolling_counter_0x239);

  // config detail: {'name': 'Checksum_0x239', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  void set_p_checksum_0x239(uint8_t* data, int checksum_0x239);

  // config detail: {'name': 'ShutdownMode', 'enum': {0: 'SHUTDOWNMODE_SOFT_OFF', 1: 'SHUTDOWNMODE_FAST_OFF', 2: 'SHUTDOWNMODE_RESERVED', 3: 'SHUTDOWNMODE_INITIAL'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_shutdownmode(uint8_t* data, Mrr_0x239_239::ShutdownmodeType shutdownmode);

  // config detail: {'name': 'ABA_Level', 'enum': {0: 'ABA_LEVEL_LEVEL_0', 1: 'ABA_LEVEL_LEVEL_1', 2: 'ABA_LEVEL_LEVEL_2', 3: 'ABA_LEVEL_LEVEL_3'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 9, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_aba_level(uint8_t* data, Mrr_0x239_239::Aba_levelType aba_level);

 private:
  Mrr_0x239_239::Acc_uppercomftbandreqType acc_uppercomftbandreq_;
  Mrr_0x239_239::Acc_brakepreferredType acc_brakepreferred_;
  Mrr_0x239_239::Eba_reqType eba_req_;
  Mrr_0x239_239::Aeb_reqType aeb_req_;
  Mrr_0x239_239::Acc_standstillreqType acc_standstillreq_;
  Mrr_0x239_239::Acc_driveoffType acc_driveoff_;
  Mrr_0x239_239::Awb_levelType awb_level_;
  Mrr_0x239_239::Abp_reqType abp_req_;
  Mrr_0x239_239::Awb_reqType awb_req_;
  Mrr_0x239_239::Aba_reqType aba_req_;
  double aeb_tgtax_;
  Mrr_0x239_239::Acc_stateType acc_state_;
  int rolling_counter_0x239_;
  int checksum_0x239_;
  Mrr_0x239_239::ShutdownmodeType shutdownmode_;
  Mrr_0x239_239::Aba_levelType aba_level_;
};

}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_cx75_PROTOCOL_MRR_0X239_239_H_
