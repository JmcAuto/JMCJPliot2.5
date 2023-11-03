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

#include "modules/canbus/vehicle/cx75/cx75_vehicle_factory.h"

#include "modules/canbus/vehicle/cx75/cx75_controller.h"
#include "modules/canbus/vehicle/cx75/cx75_message_manager.h"
#include "modules/common/log.h"
#include "modules/common/util/util.h"

namespace jmc_auto {
namespace canbus {

std::unique_ptr<VehicleController>
Cx75VehicleFactory::CreateVehicleController() {
  return std::unique_ptr<VehicleController>(new cx75::Cx75Controller());
}

std::unique_ptr<MessageManager<::jmc_auto::canbus::ChassisDetail>>
Cx75VehicleFactory::CreateMessageManager() {
  return std::unique_ptr<MessageManager<::jmc_auto::canbus::ChassisDetail>>(
	new cx75::Cx75MessageManager());
}

}  // namespace canbus
}  // namespace jmc_auto
