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

#include "modules/drivers/gnss_m2/common/m2_gflags.h"

// System gflags
DEFINE_string(m2_node_name, "m2", "The chassis module name in proto");
DEFINE_string(m2_driver_name, "m2", "Driver name.");

DEFINE_string(m2_adapter_config_filename, "modules/canbus/conf/adapter.conf",
              "The adapter config file");

// data file
DEFINE_string(m2_sensor_conf_file, "", "Sensor conf file");

// Canbus gflags
DEFINE_double(m2_sensor_freq, 20,
              "Sensor feedback timer frequency -- 0 means event trigger.");

// System gflags
DEFINE_string(m2_sensor_node_name, "", "Sensor node name.");
