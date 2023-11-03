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
 * @file %(car_type_lower)s_vehicle_factory.h
 */

#ifndef MODULES_CANBUS_VEHICLE_%(car_type_upper)s_VEHICLE_FACTORY_H_
#define MODULES_CANBUS_VEHICLE_%(car_type_upper)s_VEHICLE_FACTORY_H_

#include <memory>

#include "modules/canbus/proto/vehicle_parameter.pb.h"
#include "modules/canbus/vehicle/abstract_vehicle_factory.h"
#include "modules/canbus/vehicle/vehicle_controller.h"
#include "modules/drivers/canbus/can_comm/message_manager.h"

/**
 * @namespace jmc_auto::canbus
 * @brief jmc_auto::canbus
 */
namespace jmc_auto {
namespace canbus {

/**
 * @class %(car_type_cap)sVehicleFactory
 *
 * @brief this class is inherited from AbstractVehicleFactory. It can be used to
 * create controller and message manager for %(car_type_lower)s vehicle.
 */
class %(car_type_cap)sVehicleFactory : public AbstractVehicleFactory {
 public:
  /**
  * @brief destructor
  */
  virtual ~%(car_type_cap)sVehicleFactory() = default;

  /**
   * @brief create %(car_type_lower)s vehicle controller
   * @returns a unique_ptr that points to the created controller
   */
  std::unique_ptr<VehicleController> CreateVehicleController() override;

  /**
   * @brief create %(car_type_lower)s message manager
   * @returns a unique_ptr that points to the created message manager
   */
  std::unique_ptr<MessageManager<::jmc_auto::canbus::ChassisDetail>> 
  CreateMessageManager() override;
};

}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICLE_%(car_type_upper)s_VEHICLE_FACTORY_H_
