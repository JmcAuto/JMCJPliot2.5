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

#ifndef MODULES_CANBUS_VEHICLE_TESHUN_TESHUN_MESSAGE_MANAGER_H_
#define MODULES_CANBUS_VEHICLE_TESHUN_TESHUN_MESSAGE_MANAGER_H_

#include "modules/canbus/proto/chassis_detail.pb.h"
#include "modules/drivers/canbus/can_comm/message_manager.h"

namespace jmc_auto {
namespace canbus {
namespace teshun {

using ::jmc_auto::drivers::canbus::MessageManager;

class TeshunMessageManager
	: public MessageManager<::jmc_auto::canbus::ChassisDetail> {
 public:
  TeshunMessageManager();
  virtual ~TeshunMessageManager();
};

}  // namespace teshun
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICLE_TESHUN_TESHUN_MESSAGE_MANAGER_H_
