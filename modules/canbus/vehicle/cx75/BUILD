load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "cx75_vehicle_factory",
    srcs = [
        "cx75_vehicle_factory.cc",
    ],
    hdrs = [
        "cx75_vehicle_factory.h",
    ],
    deps = [
        ":cx75_controller",
        ":cx75_message_manager",
        "//modules/canbus/vehicle:abstract_vehicle_factory",
    ],
)

cc_library(
    name = "cx75_message_manager",
    srcs = [
        "cx75_message_manager.cc",
    ],
    hdrs = [
        "cx75_message_manager.h",
    ],
    deps = [
        "//modules/drivers/canbus/common:canbus_common",
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/canbus/vehicle/cx75/protocol:canbus_cx75_protocol",
    ],
)

cc_library(
    name = "cx75_controller",
    srcs = [
        "cx75_controller.cc",
    ],
    hdrs = [
        "cx75_controller.h",
    ],
    deps = [
        ":cx75_message_manager",
        "//modules/drivers/canbus/can_comm:can_sender",
        "//modules/drivers/canbus/common:canbus_common",
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/canbus/vehicle:vehicle_controller_base",
        "//modules/canbus/vehicle/cx75/protocol:canbus_cx75_protocol",
    ],
)

cpplint()
