#ifndef MODULES_REMOTECONTROL_COMMON_GFLAGS_H_
#define MODULES_REMOTECONTROL_COMMON_GFLAGS_H_

#include "gflags/gflags.h"

// System gflags
//DECLARE_string(canbus_node_name);
DECLARE_string(remotecontrol_module_name);

DECLARE_string(remotecontrol_adapter_config_filename);

// data file
DECLARE_string(remotecontrol_conf_file);

// // Canbus gflags
DECLARE_double(remotecontrol_freq);
// DECLARE_int64(min_cmd_interval);

// // chassis_detail message publish
// DECLARE_bool(enable_chassis_detail_pub);

// // canbus test files
// DECLARE_string(canbus_test_file);

// // canbus test files
// DECLARE_bool(receive_guardian);
DECLARE_bool(enable_csv_debug);

DECLARE_string(speed_control_conf_file);
DECLARE_double(Target_speed_high_limit);
DECLARE_double(Target_speed_low_limit);
DECLARE_bool(enable_slope_offset);
DECLARE_double(max_acceleration_when_stopped);

#endif
