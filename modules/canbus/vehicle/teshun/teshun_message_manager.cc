/* Copyright 2017 The JmcAuto Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/

#include "modules/canbus/vehicle/teshun/teshun_message_manager.h"

#include "modules/canbus/vehicle/teshun/protocol/adu_bodycontrol_0x333_333.h"
#include "modules/canbus/vehicle/teshun/protocol/adu_controlbrake_0x110_110.h"
#include "modules/canbus/vehicle/teshun/protocol/adu_controldrive_0x120_120.h"
#include "modules/canbus/vehicle/teshun/protocol/adu_controleps2_0x100_100.h"

#include "modules/canbus/vehicle/teshun/protocol/bcm_bodysts_0x344_344.h"
#include "modules/canbus/vehicle/teshun/protocol/bcm_door_0x310_310.h"
#include "modules/canbus/vehicle/teshun/protocol/epb_status_0x152_152.h"
#include "modules/canbus/vehicle/teshun/protocol/eps2_status_0x112_112.h"
#include "modules/canbus/vehicle/teshun/protocol/gw_abs_0x211_211.h"
#include "modules/canbus/vehicle/teshun/protocol/gw_abs_sts_0x221_221.h"
#include "modules/canbus/vehicle/teshun/protocol/gw_bms_display_0x323_323.h"
#include "modules/canbus/vehicle/teshun/protocol/gw_bms_sts_0x181_181.h"
#include "modules/canbus/vehicle/teshun/protocol/gw_mcu_output_0x225_225.h"
#include "modules/canbus/vehicle/teshun/protocol/gw_mcu_power_0x226_226.h"
#include "modules/canbus/vehicle/teshun/protocol/gw_scu_shiftersts_0xc8_c8.h"
#include "modules/canbus/vehicle/teshun/protocol/gw_vcu_control2_0x131_131.h"
#include "modules/canbus/vehicle/teshun/protocol/gw_vcu_control_0x185_185.h"
#include "modules/canbus/vehicle/teshun/protocol/gw_vcu_drivests_0x10a_10a.h"
#include "modules/canbus/vehicle/teshun/protocol/gw_vcu_hmi_0x358_358.h"
#include "modules/canbus/vehicle/teshun/protocol/gw_vcu_sts_0x218_218.h"
#include "modules/canbus/vehicle/teshun/protocol/gw_vcu_whltq_0x107_107.h"
#include "modules/canbus/vehicle/teshun/protocol/ibc_status2_0x124_124.h"
#include "modules/canbus/vehicle/teshun/protocol/ibc_status_0x122_122.h"

namespace jmc_auto {
namespace canbus {
namespace teshun {

TeshunMessageManager::TeshunMessageManager() {
  // Control Messages
  AddSendProtocolData<Adubodycontrol0x333333, true>();
  AddSendProtocolData<Aducontrolbrake0x110110, true>();
  AddSendProtocolData<Aducontroldrive0x120120, true>();
  AddSendProtocolData<Aducontroleps20x100100, true>();

  // Report Messages
  AddRecvProtocolData<Bcmbodysts0x344344, true>();
  AddRecvProtocolData<Bcmdoor0x310310, true>();
  AddRecvProtocolData<Epbstatus0x152152, true>();
  AddRecvProtocolData<Eps2status0x112112, true>();
  AddRecvProtocolData<Gwabs0x211211, true>();
  AddRecvProtocolData<Gwabssts0x221221, true>();
  AddRecvProtocolData<Gwbmsdisplay0x323323, true>();
  AddRecvProtocolData<Gwbmssts0x181181, true>();
  AddRecvProtocolData<Gwmcuoutput0x225225, true>();
  AddRecvProtocolData<Gwmcupower0x226226, true>();
  AddRecvProtocolData<Gwscushiftersts0xc8c8, true>();
  AddRecvProtocolData<Gwvcucontrol0x185185, true>();
  AddRecvProtocolData<Gwvcucontrol20x131131, true>();
  AddRecvProtocolData<Gwvcudrivests0x10a10a, true>();
  AddRecvProtocolData<Gwvcuhmi0x358358, true>();
  AddRecvProtocolData<Gwvcusts0x218218, true>();
  AddRecvProtocolData<Gwvcuwhltq0x107107, true>();
  AddRecvProtocolData<Ibcstatus0x122122, true>();
  AddRecvProtocolData<Ibcstatus20x124124, true>();
}

TeshunMessageManager::~TeshunMessageManager() {}

}  // namespace teshun
}  // namespace canbus
}  // namespace jmc_auto
