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

#ifndef MODULES_CANBUS_VEHICLE_cx75_PROTOCOL_GW_BODY_0X321_321_H_
#define MODULES_CANBUS_VEHICLE_cx75_PROTOCOL_GW_BODY_0X321_321_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

class Gwbody0x321321 : public ::jmc_auto::drivers::canbus::ProtocolData<
                    ::jmc_auto::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Gwbody0x321321();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'name': 'BCM_OutsideTempDig', 'enum': {0: 'BCM_OUTSIDETEMPDIG_NOFAULT', 1: 'BCM_OUTSIDETEMPDIG_SHORTCIRCUIT', 2: 'BCM_OUTSIDETEMPDIG_OPENCIRCUIT'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 1, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_body_0x321_321::Bcm_outsidetempdigType bcm_outsidetempdig(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'BCM_OutsideTemp', 'offset': -48.0, 'precision': 0.75, 'len': 8, 'is_signed_var': False, 'physical_range': '[-48|142.5]', 'bit': 15, 'type': 'double', 'order': 'motorola', 'physical_unit': 'degC'}
  double bcm_outsidetemp(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'BCM_Driving_mode_req_BCM', 'enum': {0: 'BCM_DRIVING_MODE_REQ_BCM_NORMAL', 1: 'BCM_DRIVING_MODE_REQ_BCM_SNOW', 2: 'BCM_DRIVING_MODE_REQ_BCM_NOUSED', 3: 'BCM_DRIVING_MODE_REQ_BCM_SPORT', 4: 'BCM_DRIVING_MODE_REQ_BCM_ECO', 5: 'BCM_DRIVING_MODE_REQ_BCM_4L', 6: 'BCM_DRIVING_MODE_REQ_BCM_ROCK', 7: 'BCM_DRIVING_MODE_REQ_BCM_WETMUD', 8: 'BCM_DRIVING_MODE_REQ_BCM_SAND', 9: 'BCM_DRIVING_MODE_REQ_BCM_GHAT_FORN330L', 13: 'BCM_DRIVING_MODE_REQ_BCM_MANUAL_FORN330L', 14: 'BCM_DRIVING_MODE_REQ_BCM_ATSFAULTBCM', 15: 'BCM_DRIVING_MODE_REQ_BCM_INVALID'}, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|15]', 'bit': 19, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_body_0x321_321::Bcm_driving_mode_req_bcmType bcm_driving_mode_req_bcm(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'BCM_PowerMode', 'enum': {0: 'BCM_POWERMODE_UNDEFINED', 1: 'BCM_POWERMODE_OFF', 2: 'BCM_POWERMODE_ACC', 3: 'BCM_POWERMODE_ON', 4: 'BCM_POWERMODE_START'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 23, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_body_0x321_321::Bcm_powermodeType bcm_powermode(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'BCM_FrntFogLmpSts', 'enum': {0: 'BCM_FRNTFOGLMPSTS_INVALID', 1: 'BCM_FRNTFOGLMPSTS_OFF', 2: 'BCM_FRNTFOGLMPSTS_ON', 3: 'BCM_FRNTFOGLMPSTS_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 25, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_body_0x321_321::Bcm_frntfoglmpstsType bcm_frntfoglmpsts(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'BCM_LowBeamSts', 'enum': {0: 'BCM_LOWBEAMSTS_INVALID', 1: 'BCM_LOWBEAMSTS_OFF', 2: 'BCM_LOWBEAMSTS_ON', 3: 'BCM_LOWBEAMSTS_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 27, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_body_0x321_321::Bcm_lowbeamstsType bcm_lowbeamsts(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'BCM_HighBeamSts', 'enum': {0: 'BCM_HIGHBEAMSTS_INVALID', 1: 'BCM_HIGHBEAMSTS_OFF', 2: 'BCM_HIGHBEAMSTS_ON', 3: 'BCM_HIGHBEAMSTS_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 29, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_body_0x321_321::Bcm_highbeamstsType bcm_highbeamsts(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'BCM_SS_EnableSts', 'enum': {0: 'BCM_SS_ENABLESTS_INVALID', 1: 'BCM_SS_ENABLESTS_STARTSTOPISALLOW', 2: 'BCM_SS_ENABLESTS_STARTSTOPISFORBID', 3: 'BCM_SS_ENABLESTS_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 33, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_body_0x321_321::Bcm_ss_enablestsType bcm_ss_enablests(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'BCM_RearDef_Status', 'enum': {0: 'BCM_REARDEF_STATUS_INVALID', 1: 'BCM_REARDEF_STATUS_REARDEFINACTIVE', 2: 'BCM_REARDEF_STATUS_REARDEFACTIVE', 3: 'BCM_REARDEF_STATUS_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 35, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_body_0x321_321::Bcm_reardef_statusType bcm_reardef_status(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'BCM_BrakeFluidSts', 'enum': {0: 'BCM_BRAKEFLUIDSTS_INVALID', 1: 'BCM_BRAKEFLUIDSTS_NORMAL', 2: 'BCM_BRAKEFLUIDSTS_LOW', 3: 'BCM_BRAKEFLUIDSTS_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 37, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_body_0x321_321::Bcm_brakefluidstsType bcm_brakefluidsts(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'BCM_AutolampSts', 'enum': {0: 'BCM_AUTOLAMPSTS_INVALID', 1: 'BCM_AUTOLAMPSTS_INACTIVE', 2: 'BCM_AUTOLAMPSTS_ACTIVE', 3: 'BCM_AUTOLAMPSTS_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 39, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_body_0x321_321::Bcm_autolampstsType bcm_autolampsts(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'BCM_PosLmpSts', 'enum': {0: 'BCM_POSLMPSTS_NOUESD', 1: 'BCM_POSLMPSTS_OFF', 2: 'BCM_POSLMPSTS_ON', 3: 'BCM_POSLMPSTS_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 41, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_body_0x321_321::Bcm_poslmpstsType bcm_poslmpsts(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'BCM_BGLLumLvlSetting', 'enum': {0: 'BCM_BGLLUMLVLSETTING_0PERCENT', 1: 'BCM_BGLLUMLVLSETTING_1PERCENT', 2: 'BCM_BGLLUMLVLSETTING_2PERCENT', 3: 'BCM_BGLLUMLVLSETTING_3PERCENT', 4: 'BCM_BGLLUMLVLSETTING_4PERCENT', 5: 'BCM_BGLLUMLVLSETTING_5PERCENT', 6: 'BCM_BGLLUMLVLSETTING_6PERCENT', 7: 'BCM_BGLLUMLVLSETTING_7PERCENT', 8: 'BCM_BGLLUMLVLSETTING_8PERCENT', 9: 'BCM_BGLLUMLVLSETTING_9PERCENT', 10: 'BCM_BGLLUMLVLSETTING_10PERCENT', 11: 'BCM_BGLLUMLVLSETTING_11PERCENT', 12: 'BCM_BGLLUMLVLSETTING_12PERCENT', 13: 'BCM_BGLLUMLVLSETTING_13PERCENT', 14: 'BCM_BGLLUMLVLSETTING_14PERCENT', 15: 'BCM_BGLLUMLVLSETTING_15PERCENT', 16: 'BCM_BGLLUMLVLSETTING_16PERCENT', 17: 'BCM_BGLLUMLVLSETTING_17PERCENT', 18: 'BCM_BGLLUMLVLSETTING_18PERCENT', 19: 'BCM_BGLLUMLVLSETTING_19PERCENT', 20: 'BCM_BGLLUMLVLSETTING_20PERCENT', 21: 'BCM_BGLLUMLVLSETTING_21PERCENT', 22: 'BCM_BGLLUMLVLSETTING_22PERCENT', 23: 'BCM_BGLLUMLVLSETTING_23PERCENT', 24: 'BCM_BGLLUMLVLSETTING_24PERCENT', 25: 'BCM_BGLLUMLVLSETTING_25PERCENT', 26: 'BCM_BGLLUMLVLSETTING_26PERCENT', 27: 'BCM_BGLLUMLVLSETTING_27PERCENT', 28: 'BCM_BGLLUMLVLSETTING_28PERCENT', 29: 'BCM_BGLLUMLVLSETTING_29PERCENT', 30: 'BCM_BGLLUMLVLSETTING_30PERCENT', 31: 'BCM_BGLLUMLVLSETTING_31PERCENT', 32: 'BCM_BGLLUMLVLSETTING_32PERCENT', 33: 'BCM_BGLLUMLVLSETTING_33PERCENT', 34: 'BCM_BGLLUMLVLSETTING_34PERCENT', 35: 'BCM_BGLLUMLVLSETTING_35PERCENT', 36: 'BCM_BGLLUMLVLSETTING_36PERCENT', 37: 'BCM_BGLLUMLVLSETTING_37PERCENT', 38: 'BCM_BGLLUMLVLSETTING_38PERCENT', 39: 'BCM_BGLLUMLVLSETTING_39PERCENT', 40: 'BCM_BGLLUMLVLSETTING_40PERCENT', 41: 'BCM_BGLLUMLVLSETTING_41PERCENT', 42: 'BCM_BGLLUMLVLSETTING_42PERCENT', 43: 'BCM_BGLLUMLVLSETTING_43PERCENT', 44: 'BCM_BGLLUMLVLSETTING_44PERCENT', 45: 'BCM_BGLLUMLVLSETTING_45PERCENT', 46: 'BCM_BGLLUMLVLSETTING_46PERCENT', 47: 'BCM_BGLLUMLVLSETTING_47PERCENT', 48: 'BCM_BGLLUMLVLSETTING_48PERCENT', 49: 'BCM_BGLLUMLVLSETTING_49PERCENT', 50: 'BCM_BGLLUMLVLSETTING_50PERCENT', 51: 'BCM_BGLLUMLVLSETTING_51PERCENT', 52: 'BCM_BGLLUMLVLSETTING_52PERCENT', 53: 'BCM_BGLLUMLVLSETTING_53PERCENT', 54: 'BCM_BGLLUMLVLSETTING_54PERCENT', 55: 'BCM_BGLLUMLVLSETTING_55PERCENT', 56: 'BCM_BGLLUMLVLSETTING_56PERCENT', 57: 'BCM_BGLLUMLVLSETTING_57PERCENT', 58: 'BCM_BGLLUMLVLSETTING_58PERCENT', 59: 'BCM_BGLLUMLVLSETTING_59PERCENT', 60: 'BCM_BGLLUMLVLSETTING_60PERCENT', 61: 'BCM_BGLLUMLVLSETTING_61PERCENT', 62: 'BCM_BGLLUMLVLSETTING_62PERCENT', 63: 'BCM_BGLLUMLVLSETTING_63PERCENT', 64: 'BCM_BGLLUMLVLSETTING_64PERCENT', 65: 'BCM_BGLLUMLVLSETTING_65PERCENT', 66: 'BCM_BGLLUMLVLSETTING_66PERCENT', 67: 'BCM_BGLLUMLVLSETTING_67PERCENT', 68: 'BCM_BGLLUMLVLSETTING_68PERCENT', 69: 'BCM_BGLLUMLVLSETTING_69PERCENT', 70: 'BCM_BGLLUMLVLSETTING_70PERCENT', 71: 'BCM_BGLLUMLVLSETTING_71PERCENT', 72: 'BCM_BGLLUMLVLSETTING_72PERCENT', 73: 'BCM_BGLLUMLVLSETTING_73PERCENT', 74: 'BCM_BGLLUMLVLSETTING_74PERCENT', 75: 'BCM_BGLLUMLVLSETTING_75PERCENT', 76: 'BCM_BGLLUMLVLSETTING_76PERCENT', 77: 'BCM_BGLLUMLVLSETTING_77PERCENT', 78: 'BCM_BGLLUMLVLSETTING_78PERCENT', 79: 'BCM_BGLLUMLVLSETTING_79PERCENT', 80: 'BCM_BGLLUMLVLSETTING_80PERCENT', 81: 'BCM_BGLLUMLVLSETTING_81PERCENT', 82: 'BCM_BGLLUMLVLSETTING_82PERCENT', 83: 'BCM_BGLLUMLVLSETTING_83PERCENT', 84: 'BCM_BGLLUMLVLSETTING_84PERCENT', 85: 'BCM_BGLLUMLVLSETTING_85PERCENT', 86: 'BCM_BGLLUMLVLSETTING_86PERCENT', 87: 'BCM_BGLLUMLVLSETTING_87PERCENT', 88: 'BCM_BGLLUMLVLSETTING_88PERCENT', 89: 'BCM_BGLLUMLVLSETTING_89PERCENT', 90: 'BCM_BGLLUMLVLSETTING_90PERCENT', 91: 'BCM_BGLLUMLVLSETTING_91PERCENT', 92: 'BCM_BGLLUMLVLSETTING_92PERCENT', 93: 'BCM_BGLLUMLVLSETTING_93PERCENT', 94: 'BCM_BGLLUMLVLSETTING_94PERCENT', 95: 'BCM_BGLLUMLVLSETTING_95PERCENT', 96: 'BCM_BGLLUMLVLSETTING_96PERCENT', 97: 'BCM_BGLLUMLVLSETTING_97PERCENT', 98: 'BCM_BGLLUMLVLSETTING_98PERCENT', 99: 'BCM_BGLLUMLVLSETTING_99PERCENT', 100: 'BCM_BGLLUMLVLSETTING_100PERCENT'}, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|255]', 'bit': 55, 'type': 'enum', 'order': 'motorola', 'physical_unit': '%'}
  Gw_body_0x321_321::Bcm_bgllumlvlsettingType bcm_bgllumlvlsetting(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'BCM_reverse_gear', 'enum': {0: 'BCM_REVERSE_GEAR_UNDEFINED', 1: 'BCM_REVERSE_GEAR_NOTINREVERSEGEAR', 2: 'BCM_REVERSE_GEAR_INREVERSEGEAR', 3: 'BCM_REVERSE_GEAR_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_body_0x321_321::Bcm_reverse_gearType bcm_reverse_gear(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_cx75_PROTOCOL_GW_BODY_0X321_321_H_
