load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "control_gflags",
    srcs = [
        "control_gflags.cc",
    ],
    hdrs = [
        "control_gflags.h",
    ],
    deps = [
        "//external:gflags",
    ],
)

cc_library(
    name = "hysteresis_filter",
    srcs = [
        "hysteresis_filter.cc",
    ],
    hdrs = [
        "hysteresis_filter.h",
    ],
)

cc_library(
    name = "interpolation_1d",
    srcs = [
        "interpolation_1d.cc",
    ],
    hdrs = [
        "interpolation_1d.h",
    ],
    deps = [
        "//modules/common:log",
        "@eigen//:eigen",
    ],
)

cc_library(
    name = "interpolation_2d",
    srcs = [
        "interpolation_2d.cc",
    ],
    hdrs = [
        "interpolation_2d.h",
    ],
    deps = [
        "//modules/common:log",
        "@eigen//:eigen",
    ],
)

cc_library(
    name = "pid_controller",
    srcs = [
        "pid_controller.cc",
    ],
    hdrs = [
        "pid_controller.h",
    ],
    deps = [
        "//modules/common:log",
        "//modules/control/proto:control_proto",
    ],
)
cc_library(
    name = "pid_increment",
    srcs = [
        "pid_increment.cc",
    ],
    hdrs = [
        "pid_increment.h",
    ],
    deps = [
        "//modules/common:log",
        "//modules/control/proto:control_proto",
    ],
)

cc_library(
    name = "trajectory_analyzer",
    srcs = [
        "trajectory_analyzer.cc",
    ],
    hdrs = [
        "trajectory_analyzer.h",
    ],
    deps = [
        "//modules/common:log",
        "//modules/common/math:linear_interpolation",
        "//modules/common/math:search",
        "//modules/common/proto:pnc_point_proto",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/planning/proto:planning_proto",
    ],
)

cc_library(
    name = "common",
    deps = [
        ":control_gflags",
        ":hysteresis_filter",
        ":interpolation_1d",
        ":interpolation_2d",
        ":pid_controller",
        ":trajectory_analyzer",
        ":pid_increment",
    ],
)

cpplint()
