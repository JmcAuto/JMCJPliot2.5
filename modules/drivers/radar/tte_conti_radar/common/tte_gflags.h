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

#ifndef MODULES_DRIVERS_RADAR_TTE_CONTI_RADAR_TTE_GFLAGS_H_
#define MODULES_DRIVERS_RADAR_TTE_CONTI_RADAR_TTE_GFLAGS_H_

#include "gflags/gflags.h"

// System gflags
DECLARE_string(tte_node_name);
DECLARE_string(tte_driver_name);

DECLARE_string(tte_adapter_config_filename);

// data file
DECLARE_string(tte_sensor_conf_file);

// Sensor gflags
DECLARE_double(tte_sensor_freq);

// System gflags
DECLARE_string(tte_sensor_node_name);
//flitter paramer
DECLARE_double(apafrm_distance_outliner);
DECLARE_int32(apafrm_distance_limittimes);

DECLARE_double(apafrs_distance_outliner);
DECLARE_int32(apafrs_distance_limittimes);

DECLARE_double(apafr_distance_outliner);
DECLARE_int32(apafr_distance_limittimes);

DECLARE_double(apafls_distance_outliner);
DECLARE_int32(apafls_distance_limittimes);

DECLARE_double(apaflm_distance_outliner);
DECLARE_int32(apaflm_distance_limittimes);

DECLARE_double(apafl_distance_outliner);
DECLARE_int32(apafl_distance_limittimes);

//
DECLARE_double(aparrs_distance_outliner);
DECLARE_int32(aparrs_distance_limittimes);

DECLARE_double(aparrm_distance_outliner);
DECLARE_int32(aparrm_distance_limittimes);

DECLARE_double(aparr_distance_outliner);
DECLARE_int32(aparr_distance_limittimes);

DECLARE_double(aparls_distance_outliner);
DECLARE_int32(aparls_distance_limittimes);

DECLARE_double(aparl_distance_outliner);
DECLARE_int32(aparl_distance_limittimes);

DECLARE_double(aparlm_distance_outliner);
DECLARE_int32(aparlm_distance_limittimes);

#endif
