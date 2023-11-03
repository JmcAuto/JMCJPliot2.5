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

/**
 * @file conti_radar_message_manager.h
 * @brief The class of ContiRadarMessageManager
 */

#include "modules/drivers/radar/tte_conti_radar/conti_radar_message_manager.h"

#include "modules/drivers/radar/tte_conti_radar/protocol/debug_extended_mode_736.h"
#include "modules/drivers/radar/tte_conti_radar/protocol/debug_parkingtype_533.h"

#include "modules/drivers/radar/tte_conti_radar/protocol/debug_apafrontdistanceinfo_458.h"
#include "modules/drivers/radar/tte_conti_radar/protocol/debug_apareardistanceinfo_457.h"
#include "modules/drivers/radar/tte_conti_radar/protocol/debug_leftusslot_lengthdepth_46a.h"
#include "modules/drivers/radar/tte_conti_radar/protocol/debug_leftusslot_ptab_469.h"
#include "modules/drivers/radar/tte_conti_radar/protocol/debug_leftvplslot_ptab_465.h"
#include "modules/drivers/radar/tte_conti_radar/protocol/debug_leftvplslot_ptcd_466.h"
#include "modules/drivers/radar/tte_conti_radar/protocol/debug_oddataoutput01_46b.h"
#include "modules/drivers/radar/tte_conti_radar/protocol/debug_oddataoutput23_46c.h"
#include "modules/drivers/radar/tte_conti_radar/protocol/debug_oddataoutput45_46d.h"
#include "modules/drivers/radar/tte_conti_radar/protocol/debug_oddataoutput67_46e.h"
#include "modules/drivers/radar/tte_conti_radar/protocol/debug_oddataoutput89_46f.h"
#include "modules/drivers/radar/tte_conti_radar/protocol/debug_oddataoutputab_45f.h"
#include "modules/drivers/radar/tte_conti_radar/protocol/debug_oddataoutputcd_461.h"
#include "modules/drivers/radar/tte_conti_radar/protocol/debug_oddataoutputef_462.h"
#include "modules/drivers/radar/tte_conti_radar/protocol/debug_parkingdetection_271.h"
#include "modules/drivers/radar/tte_conti_radar/protocol/debug_rightusslot_lengthdepth_468.h"
#include "modules/drivers/radar/tte_conti_radar/protocol/debug_rightusslot_ptab_467.h"
#include "modules/drivers/radar/tte_conti_radar/protocol/debug_rightvplslot_ptab_463.h"
#include "modules/drivers/radar/tte_conti_radar/protocol/debug_rightvplslot_ptcd_464.h"
#include "modules/drivers/radar/tte_conti_radar/protocol/debug_ttemodeinfo73e_73e.h"

namespace jmc_auto {
namespace drivers {
namespace tte_conti_radar {

using ::jmc_auto::common::adapter::AdapterManager;

ContiRadarMessageManager::ContiRadarMessageManager() {
  // Control Messages
  AddSendProtocolData<Debugextendedmode736, true>();//event
  AddSendProtocolData<Debugparkingtype533, true>();

  // Report Messages
  AddRecvProtocolData<Debugapafrontdistanceinfo458, true>();
  AddRecvProtocolData<Debugapareardistanceinfo457, true>();
  AddRecvProtocolData<Debugleftusslotlengthdepth46a, true>();
  AddRecvProtocolData<Debugleftusslotptab469, true>();
  AddRecvProtocolData<Debugleftvplslotptab465, true>();
  AddRecvProtocolData<Debugleftvplslotptcd466, true>();
  AddRecvProtocolData<Debugoddataoutput0146b, true>();
  AddRecvProtocolData<Debugoddataoutput2346c, true>();
  AddRecvProtocolData<Debugoddataoutput4546d, true>();
  AddRecvProtocolData<Debugoddataoutput6746e, true>();
  AddRecvProtocolData<Debugoddataoutput8946f, true>();
  AddRecvProtocolData<Debugoddataoutputab45f, true>();
  AddRecvProtocolData<Debugoddataoutputcd461, true>();
  AddRecvProtocolData<Debugoddataoutputef462, true>();
  AddRecvProtocolData<Debugparkingdetection271, true>();
  AddRecvProtocolData<Debugrightusslotlengthdepth468, true>();
  AddRecvProtocolData<Debugrightusslotptab467, true>();
  AddRecvProtocolData<Debugrightvplslotptab463, true>();
  AddRecvProtocolData<Debugrightvplslotptcd464, true>();
  AddRecvProtocolData<Debugttemodeinfo73e73e, true>();
}

}  // namespace conti_radar
}  // namespace drivers
}  // namespace jmc_auto
