load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "remotecontrol_common",
    srcs = [
        "remotecontrol_gflags.cc",
    ],
    hdrs = [
        "remotecontrol_gflags.h",
    ],
    deps = [
        "//external:gflags",
    ],
)
cc_library(
    name = "speed_control",
    srcs = [
        "speed_control.cc",
    ],
    hdrs = [
        "speed_control.h",
    ],
    deps = [
        "//modules/common:log",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/filters:digital_filter",
        "//modules/common/filters:digital_filter_coefficients",
        "//modules/common/status",
        "//modules/common/time",
        "//modules/common/vehicle_state:vehicle_state_provider",
        ":remotecontrol_common",
        "//modules/control/common:interpolation_2d",
        "//modules/control/common:pid_controller",
        "//modules/control/proto:control_proto",
        "//modules/common/util:util",
        "//modules/remotecontrol/proto:remotecontrol_proto",
    ],
)
cpplint()