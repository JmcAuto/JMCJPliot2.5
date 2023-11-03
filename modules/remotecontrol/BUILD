
package(default_visibility = ["//visibility:public"])

cc_library(
    name = "remotecontrol_lib",
    srcs = ["remotecontrol.cc"],
    hdrs = ["remotecontrol.h"],
    deps = [
        "//modules/remotecontrol/udp_stream:udp_data",
        "@com_google_protobuf//:protobuf",
        "//modules/common",
        "//modules/common:jmc_auto_app",
        "//modules/common/adapters:adapter_manager",
        "//modules/common/monitor_log",
        "//modules/canbus/proto:canbus_proto",
        "//modules/control/proto:control_proto",
        "//modules/remotecontrol/proto:remotecontrol_proto",
        "//modules/remotecontrol/common:remotecontrol_common",
        "//modules/common/time",
        "//modules/common/util",
        "//modules/common:log",
        "//modules/remotecontrol/common:speed_control",
        "@glog",
    ],
)

cc_binary(
    name = "remotecontrol",
    srcs = ["main.cc"],
    deps = [
        ":remotecontrol_lib",
        "//external:gflags",
        "@ros//:ros_common",
    ],
)

