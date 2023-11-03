load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "discretized_trajectory",
    srcs = ["discretized_trajectory.cc"],
    hdrs = ["discretized_trajectory.h"],
    deps = [
        "//modules/common/math:linear_interpolation",
        "//modules/common/proto:pnc_point_proto",
        "//modules/common/vehicle_state/proto:vehicle_state_proto",
        "//modules/planning/common:planning_context",
        "@eigen",
    ],
)


cc_library(
    name = "publishable_trajectory",
    srcs = ["publishable_trajectory.cc"],
    hdrs = ["publishable_trajectory.h"],
    deps = [
        ":discretized_trajectory",
        "//modules/planning/proto:planning_proto",
    ],
)

cpplint()
