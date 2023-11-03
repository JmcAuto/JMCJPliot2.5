load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "teshun_vehicle_factory",
    srcs = [
        "teshun_vehicle_factory.cc",
    ],
    hdrs = [
        "teshun_vehicle_factory.h",
    ],
    deps = [
        ":teshun_controller",
        ":teshun_message_manager",
        "//modules/canbus/vehicle:abstract_vehicle_factory",
    ],
)

cc_library(
    name = "teshun_message_manager",
    srcs = [
        "teshun_message_manager.cc",
    ],
    hdrs = [
        "teshun_message_manager.h",
    ],
    deps = [
        "//modules/drivers/canbus/common:canbus_common",
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/canbus/vehicle/teshun/protocol:canbus_teshun_protocol",
    ],
)

cc_library(
    name = "teshun_controller",
    srcs = [
        "teshun_controller.cc",
    ],
    hdrs = [
        "teshun_controller.h",
    ],
    deps = [
        ":teshun_message_manager",
        "//modules/drivers/canbus/can_comm:can_sender",
        "//modules/drivers/canbus/common:canbus_common",
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/canbus/vehicle:vehicle_controller_base",
        "//modules/canbus/vehicle/teshun/protocol:canbus_teshun_protocol",
    ],
)

cpplint()
