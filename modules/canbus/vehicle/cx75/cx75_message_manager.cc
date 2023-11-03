/* Copyright 2017 The jmc_auto Authors. All Rights Reserved.

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

#include "modules/canbus/vehicle/cx75/cx75_message_manager.h"

#include "modules/canbus/vehicle/cx75/protocol/pam_0x270_270.h"
#include "modules/canbus/vehicle/cx75/protocol/pam_0x271_271.h"
#include "modules/canbus/vehicle/cx75/protocol/pam_0x272_272.h"

#include "modules/canbus/vehicle/cx75/protocol/abs_sts_0x221_221.h"
#include "modules/canbus/vehicle/cx75/protocol/eps_advanced_0x176_176.h"
#include "modules/canbus/vehicle/cx75/protocol/esp_advanced_0x234_234.h"
#include "modules/canbus/vehicle/cx75/protocol/esp_axay_0x242_242.h"
#include "modules/canbus/vehicle/cx75/protocol/esp_direction_0x235_235.h"
#include "modules/canbus/vehicle/cx75/protocol/esp_pressure_0x241_241.h"
#include "modules/canbus/vehicle/cx75/protocol/esp_raw_0x212_212.h"
#include "modules/canbus/vehicle/cx75/protocol/esp_status_0x243_243.h"
#include "modules/canbus/vehicle/cx75/protocol/esp_vlc_0x223_223.h"
#include "modules/canbus/vehicle/cx75/protocol/esp_whlpulse_0x236_236.h"
#include "modules/canbus/vehicle/cx75/protocol/gw_bcm_door_0x310_310.h"
#include "modules/canbus/vehicle/cx75/protocol/gw_body_0x321_321.h"
#include "modules/canbus/vehicle/cx75/protocol/gw_ems_engstatus_0x142_142.h"
#include "modules/canbus/vehicle/cx75/protocol/gw_ems_sts_0x151_151.h"
#include "modules/canbus/vehicle/cx75/protocol/gw_ems_tq_0x101_101.h"
#include "modules/canbus/vehicle/cx75/protocol/gw_ems_tqwhl_0x111_111.h"
#include "modules/canbus/vehicle/cx75/protocol/gw_ems_whltq_0x107_107.h"
#include "modules/canbus/vehicle/cx75/protocol/gw_ic_0x510_510.h"
#include "modules/canbus/vehicle/cx75/protocol/gw_mp5_0x530_530.h"
#include "modules/canbus/vehicle/cx75/protocol/gw_mp5_nav_0x533_533.h"
#include "modules/canbus/vehicle/cx75/protocol/gw_swm_body_0x31a_31a.h"
#include "modules/canbus/vehicle/cx75/protocol/gw_swm_mrr_0x31b_31b.h"
#include "modules/canbus/vehicle/cx75/protocol/gw_tbox_location_0x580_580.h"
#include "modules/canbus/vehicle/cx75/protocol/gw_tcu_gearinfo_0x123_123.h"
#include "modules/canbus/vehicle/cx75/protocol/ipm_0x245_245.h"
#include "modules/canbus/vehicle/cx75/protocol/ipm_leftline_0x278_278.h"
#include "modules/canbus/vehicle/cx75/protocol/ipm_rightline_0x490_490.h"
#include "modules/canbus/vehicle/cx75/protocol/mrr_0x238_238.h"
#include "modules/canbus/vehicle/cx75/protocol/mrr_0x239_239.h"
#include "modules/canbus/vehicle/cx75/protocol/mrr_0x246_246.h"
#include "modules/canbus/vehicle/cx75/protocol/mrr_frobj_0x279_279.h"
#include "modules/canbus/vehicle/cx75/protocol/mrr_frobj_0x480_480.h"
#include "modules/canbus/vehicle/cx75/protocol/sas_sensor_0x175_175.h"
#include "modules/canbus/vehicle/cx75/protocol/srs_0x350_350.h"

#include "modules/canbus/vehicle/cx75/protocol/ins_datainfo_506.h"
#include "modules/canbus/vehicle/cx75/protocol/ins_gyro_501.h"
#include "modules/canbus/vehicle/cx75/protocol/ins_headingpitchroll_502.h"
#include "modules/canbus/vehicle/cx75/protocol/ins_heightandtime_503.h"
#include "modules/canbus/vehicle/cx75/protocol/ins_latitudelongitude_504.h"
#include "modules/canbus/vehicle/cx75/protocol/ins_speed_505.h"
#include "modules/canbus/vehicle/cx75/protocol/ins_std_507.h"
#include "modules/canbus/vehicle/cx75/protocol/ins_acc_500.h"





namespace jmc_auto {
namespace canbus {
namespace cx75 {

Cx75MessageManager::Cx75MessageManager() {
  // Control Messages
  AddSendProtocolData<Mrr0x238238, true>();
  AddSendProtocolData<Mrr0x239239, true>();
  AddSendProtocolData<Mrr0x246246, true>();
  AddSendProtocolData<Mrrfrobj0x279279, true>();
  AddSendProtocolData<Mrrfrobj0x480480, true>();
  AddSendProtocolData<Ipm0x245245, true>();
  AddSendProtocolData<Ipmleftline0x278278, true>();
  AddSendProtocolData<Ipmrightline0x490490, true>();
  AddSendProtocolData<Pam0x270270, true>();
  AddSendProtocolData<Pam0x271271, true>();
  AddSendProtocolData<Pam0x272272, true>();

  // Report Messages
  AddRecvProtocolData<Abssts0x221221, true>();
  AddRecvProtocolData<Epsadvanced0x176176, true>();
  AddRecvProtocolData<Espadvanced0x234234, true>();
  AddRecvProtocolData<Espaxay0x242242, true>();
  AddRecvProtocolData<Espdirection0x235235, true>();
  AddRecvProtocolData<Esppressure0x241241, true>();
  AddRecvProtocolData<Espraw0x212212, true>();
  AddRecvProtocolData<Espstatus0x243243, true>();
  AddRecvProtocolData<Espvlc0x223223, true>();
  AddRecvProtocolData<Espwhlpulse0x236236, true>();
  AddRecvProtocolData<Gwbcmdoor0x310310, true>();
  AddRecvProtocolData<Gwbody0x321321, true>();
  AddRecvProtocolData<Gwemsengstatus0x142142, true>();
  AddRecvProtocolData<Gwemssts0x151151, true>();
  AddRecvProtocolData<Gwemstq0x101101, true>();
  AddRecvProtocolData<Gwemstqwhl0x111111, true>();
  AddRecvProtocolData<Gwemswhltq0x107107, true>();
  AddRecvProtocolData<Gwic0x510510, true>();
  AddRecvProtocolData<Gwmp50x530530, true>();
  //AddRecvProtocolData<Gwmp5nav0x533533, true>();
  AddRecvProtocolData<Gwswmbody0x31a31a, true>();
  AddRecvProtocolData<Gwswmmrr0x31b31b, true>();
  AddRecvProtocolData<Gwtboxlocation0x580580, true>();
  AddRecvProtocolData<Gwtcugearinfo0x123123, true>();
  AddRecvProtocolData<Sassensor0x175175, true>();
  AddRecvProtocolData<Srs0x350350, true>();

  AddRecvProtocolData<Insacc500, true>();
  AddRecvProtocolData<Insdatainfo506, true>();
  AddRecvProtocolData<Insgyro501, true>();
  AddRecvProtocolData<Insheadingpitchroll502, true>();
  AddRecvProtocolData<Insheightandtime503, true>();
  AddRecvProtocolData<Inslatitudelongitude504, true>();
  AddRecvProtocolData<Insspeed505, true>();
  AddRecvProtocolData<Insstd507, true>();
}
  Cx75MessageManager::~Cx75MessageManager() {}



}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto
