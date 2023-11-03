load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "gnss_m2_message_manager",
    srcs = [
        "gnss_m2_message_manager.cc",
    ],
    hdrs = [
        "gnss_m2_message_manager.h",
    ],
    deps = [
        "//modules/common/adapters:adapter_manager",
        "//modules/drivers/canbus:sensor_gflags",
        "//modules/drivers/canbus/can_client:can_client_factory",
        "//modules/drivers/canbus/can_comm:can_sender",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/gnss_m2/protocol:canbus_m2_protocol",
        "//modules/drivers/gnss_m2/proto:m2_proto",
    ],
)

cc_library(
    name = "gnss_m2_canbus_lib",
    srcs = [
        "gnss_m2_canbus.cc",
    ],
    hdrs = [
        "gnss_m2_canbus.h",
    ],
    deps = [
        ":gnss_m2_message_manager",
        "//modules/common",
        "//modules/common:jmc_auto_app",
        "//modules/common/adapters:adapter_manager",
        "//modules/common/monitor_log",
        "//modules/drivers/gnss_m2/common:gnss_m2_common",
        "//modules/drivers/canbus/can_client:can_client_factory",
        "//modules/drivers/canbus/can_comm:can_receiver",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/gnss_m2/protocol:canbus_m2_protocol",
        "//modules/drivers/gnss_m2/proto:m2_proto",
        "//modules/drivers/gnss/proto:gnss_proto",
        
    ],
)

cc_binary(
    name = "gnss_m2",
    srcs = ["main.cc"],
    deps = [
        ":gnss_m2_canbus_lib",
        "//external:gflags",
        "//modules/common:log",
        "//modules/common/monitor_log",
        "//modules/drivers/canbus/common:canbus_common",
        "@ros//:ros_common",
    ],
)

cpplint()
