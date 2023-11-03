#include "modules/remotecontrol/common/remotecontrol_gflags.h"

// System gflags
//DEFINE_string(canbus_node_name, "chassis", "The chassis module name in proto");
DEFINE_string(remotecontrol_module_name, "remotecontrol", "Module name");

DEFINE_string(remotecontrol_adapter_config_filename,
              "modules/remotecontrol/conf/adapter.conf", "The adapter config file");

// data file
DEFINE_string(remotecontrol_conf_file, "modules/remotecontrol/conf/remotecontrol_conf.pb.txt",
              "Default remotecontrol conf file");
// // Canbus gflags
DEFINE_double(remotecontrol_freq, 100, "remotecontrol feedback timer frequency.");
// DEFINE_int64(min_cmd_interval, 5, "Minimum control command interval in ms.");

// // chassis_detail message publish
// DEFINE_bool(enable_chassis_detail_pub, false, "Chassis Detail message publish");

// // canbus test files
// DEFINE_string(canbus_test_file, "modules/canbus/testdata/canbus_test.pb.txt",
//               "canbus tester input test file, in ControlCommand pb format.");

// // enable receiving guardian command
// // TODO(QiL) : depreciate this after test
// DEFINE_bool(receive_guardian, false,
//             "Enable receiving guardian message on canbus side");
DEFINE_bool(enable_csv_debug, false,
            "Enable save speed_log.csv");
DEFINE_string(speed_control_conf_file, "modules/control/conf/vehicle.pb.txt",
              "Default remotecontrol conf file");
DEFINE_double(Target_speed_high_limit, 0.2, " target speed Maximum limit.");
DEFINE_double(Target_speed_low_limit, 0.2, "target speed Minimum limit.");
DEFINE_bool(enable_slope_offset, false, "Enable slope offset compensation");
DEFINE_double(max_acceleration_when_stopped, 0.01,
              "max acceleration can be observed when vehicle is stopped");