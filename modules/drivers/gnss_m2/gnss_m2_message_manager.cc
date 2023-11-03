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

#include "modules/drivers/gnss_m2/gnss_m2_message_manager.h"

#include "modules/drivers/gnss_m2/protocol/odometer_input_99.h"

#include "modules/drivers/gnss_m2/protocol/combinealtitude_704.h"
#include "modules/drivers/gnss_m2/protocol/combineattitude_706.h"
#include "modules/drivers/gnss_m2/protocol/combinegpstime_701.h"
#include "modules/drivers/gnss_m2/protocol/combineposition_703.h"
#include "modules/drivers/gnss_m2/protocol/combinestatus_710.h"
#include "modules/drivers/gnss_m2/protocol/combineutctime_702.h"
#include "modules/drivers/gnss_m2/protocol/combinevelocity_705.h"
#include "modules/drivers/gnss_m2/protocol/imu_1_721.h"
#include "modules/drivers/gnss_m2/protocol/imu_2_722.h"
#include "modules/drivers/gnss_m2/protocol/imu_3_723.h"
#include "modules/drivers/gnss_m2/protocol/revraltitude_734.h"
#include "modules/drivers/gnss_m2/protocol/revrattitude_736.h"
#include "modules/drivers/gnss_m2/protocol/revrgpstime_731.h"
#include "modules/drivers/gnss_m2/protocol/revrposition_733.h"
#include "modules/drivers/gnss_m2/protocol/revrstatus_737.h"
#include "modules/drivers/gnss_m2/protocol/revrutctime_732.h"
#include "modules/drivers/gnss_m2/protocol/revrvelocity_735.h"
#include "modules/drivers/gnss_m2/protocol/stdattitude_709.h"
#include "modules/drivers/gnss_m2/protocol/stdposition_707.h"
#include "modules/drivers/gnss_m2/protocol/stdvelocity_708.h"

namespace jmc_auto {
namespace drivers {
namespace gnss_m2 {

using ::jmc_auto::common::adapter::AdapterManager;

GnssM2MessageManager::GnssM2MessageManager() {
  // Control Messages
  AddSendProtocolData<Odometerinput99, true>();

  // Report Messages
  AddRecvProtocolData<Combinealtitude704, true>();
  AddRecvProtocolData<Combineattitude706, true>();
  AddRecvProtocolData<Combinegpstime701, true>();
  AddRecvProtocolData<Combineposition703, true>();
  AddRecvProtocolData<Combinestatus710, true>();
  AddRecvProtocolData<Combineutctime702, true>();
  AddRecvProtocolData<Combinevelocity705, true>();
  AddRecvProtocolData<Imu1721, true>();
  AddRecvProtocolData<Imu2722, true>();
  AddRecvProtocolData<Imu3723, true>();
  AddRecvProtocolData<Revraltitude734, true>();
  AddRecvProtocolData<Revrattitude736, true>();
  AddRecvProtocolData<Revrgpstime731, true>();
  AddRecvProtocolData<Revrposition733, true>();
  AddRecvProtocolData<Revrstatus737, true>();
  AddRecvProtocolData<Revrutctime732, true>();
  AddRecvProtocolData<Revrvelocity735, true>();
  AddRecvProtocolData<Stdattitude709, true>();
  AddRecvProtocolData<Stdposition707, true>();
  AddRecvProtocolData<Stdvelocity708, true>();
}

}  // namespace conti_radar
}  // namespace drivers
}  // namespace jmc_auto
