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
 * @file
 */

#ifndef MODULES_DRIVERS_CANBUS_COMMON_CANBUS_CONSTS_H_
#define MODULES_DRIVERS_CANBUS_COMMON_CANBUS_CONSTS_H_

#include <cstdint>

/**
 * @namespace jmc_auto::drivers::canbus
 * @brief jmc_auto::drivers::canbus
 */
namespace jmc_auto {
namespace drivers {
namespace canbus {

const int32_t CAN_FRAME_SIZE = 8;
const int32_t MAX_CAN_SEND_FRAME_LEN = 1;
const int32_t MAX_CAN_RECV_FRAME_LEN = 10;

const int32_t CANBUS_MESSAGE_LENGTH = 8;  // according to ISO-11891-1
const int32_t MAX_CAN_PORT = 3;

}  // namespace canbus
}  // namespace drivers
}  // namespace jmc_auto

#endif  // MODULES_DRIVERS_CANBUS_COMMON_CANBUS_CONSTS_H_
