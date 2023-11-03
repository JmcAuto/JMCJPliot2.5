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

#include "modules/drivers/radar/tte_conti_radar/common/tte_gflags.h"

// System gflags
DEFINE_string(tte_node_name, "tte", "The chassis module name in proto");
DEFINE_string(tte_driver_name, "tte", "Driver name.");

DEFINE_string(tte_adapter_config_filename, "modules/canbus/conf/adapter.conf",
              "The adapter config file");

// data file
DEFINE_string(tte_sensor_conf_file, "", "Sensor conf file");

// Canbus gflags
DEFINE_double(tte_sensor_freq, 20,
              "Sensor feedback timer frequency -- 0 means event trigger.");

// System gflags
DEFINE_string(tte_sensor_node_name, "", "Sensor node name.");

//flitter paramer
DEFINE_double(apafrm_distance_outliner, 510.0, "apafrm_distance_outliner");
DEFINE_int32(apafrm_distance_limittimes,5, "apafrm_distance_limittimes");

DEFINE_double(apafrs_distance_outliner, 32.7670, "apafrm_distance_outliner");
DEFINE_int32(apafrs_distance_limittimes,5, "apafrm_distance_limittimes");

DEFINE_double(apafr_distance_outliner, 510.0, "apafrm_distance_outliner");
DEFINE_int32(apafr_distance_limittimes,5, "apafrm_distance_limittimes");

DEFINE_double(apafls_distance_outliner, 32.7670, "apafrm_distance_outliner");
DEFINE_int32(apafls_distance_limittimes,5, "apafrm_distance_limittimes");

DEFINE_double(apaflm_distance_outliner, 510.0, "apafrm_distance_outliner");
DEFINE_int32(apaflm_distance_limittimes,5, "apafrm_distance_limittimes");

DEFINE_double(apafl_distance_outliner, 510.0, "apafrm_distance_outliner");
DEFINE_int32(apafl_distance_limittimes,5, "apafrm_distance_limittimes");
//flitter paramer
DEFINE_double(aparrs_distance_outliner, 32.7670, "apafrm_distance_outliner");
DEFINE_int32(aparrs_distance_limittimes,5, "apafrm_distance_limittimes");

DEFINE_double(aparrm_distance_outliner, 510.0, "apafrm_distance_outliner");
DEFINE_int32(aparrm_distance_limittimes,5, "apafrm_distance_limittimes");

DEFINE_double(aparr_distance_outliner, 510.0, "apafrm_distance_outliner");
DEFINE_int32(aparr_distance_limittimes,5, "apafrm_distance_limittimes");

DEFINE_double(aparls_distance_outliner, 32.7670, "apafrm_distance_outliner");
DEFINE_int32(aparls_distance_limittimes,5, "apafrm_distance_limittimes");

DEFINE_double(aparl_distance_outliner, 510.0, "apafrm_distance_outliner");
DEFINE_int32(aparl_distance_limittimes,5, "apafrm_distance_limittimes");

DEFINE_double(aparlm_distance_outliner, 510.0, "apafrm_distance_outliner");
DEFINE_int32(aparlm_distance_limittimes,5, "apafrm_distance_limittimes");
