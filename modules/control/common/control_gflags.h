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

#ifndef MODULES_CONTROL_COMMON_CONTROL_GFLAGS_H_
#define MODULES_CONTROL_COMMON_CONTROL_GFLAGS_H_

#include "gflags/gflags.h"

// data file
DECLARE_string(control_conf_file);

DECLARE_double(control_test_duration);

DECLARE_string(control_adapter_config_filename);

DECLARE_bool(enable_csv_debug);

// temporary gflag for test purpose
DECLARE_bool(enable_speed_station_preview);

DECLARE_string(control_node_name);
DECLARE_bool(is_control_test_mode);
//DECLARE_bool(use_preview_speed_for_table);

DECLARE_bool(enable_input_timestamp_check);

DECLARE_int32(max_localization_miss_num);
DECLARE_int32(max_chassis_miss_num);
DECLARE_int32(max_planning_miss_num);
DECLARE_int32(planning_zero_velocity_num);


DECLARE_double(max_acceleration_when_stopped);

DECLARE_double(steer_angle_rate);
DECLARE_bool(enable_gain_scheduler);
DECLARE_bool(set_steer_limit);
DECLARE_bool(enable_slope_offset);

DECLARE_double(lock_steer_speed);

//DECLARE_bool(enable_navigation_mode_handlilng);
DECLARE_bool(enable_persistent_estop);
DECLARE_bool(enable_steering_calibration_compensate);
DECLARE_double(steering_calibration_coeff);
DECLARE_double(max_abs_speed_when_stopped);
DECLARE_double(stop_path_remain);

//5.5
DECLARE_bool(reverse_heading_control);
DECLARE_bool(trajectory_transform_to_com_reverse);
DECLARE_bool(trajectory_transform_to_com_drive);

DECLARE_bool(enable_maximum_steer_rate_limit);

DECLARE_bool(query_time_nearest_point_only);
DECLARE_bool(query_forward_time_point_only);

DECLARE_bool(enable_feedback_augment_on_high_speed);

DECLARE_bool(enable_gear_drive_negative_speed_protection);

DECLARE_bool(use_dynamic_model);
DECLARE_double(drive_pure_distance);
DECLARE_double(reverse_pure_distance);
DECLARE_bool(enable_lateral_preview);
//RADAR SAFE DISTANCE
DECLARE_double(safe_distance_level_0);
DECLARE_double(safe_distance_level_1);
DECLARE_double(safe_distance_level_2);
DECLARE_double(safe_distance_level_3);
DECLARE_double(safe_distance_level_4);
DECLARE_double(safe_distance_level_5);

DECLARE_bool(enable_last_distance_by_wheelpulse);

#endif  // MODULES_CONTROL_COMMON_CONTROL_GFLAGS_H_
