load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "open_space_roi_decider",
    srcs = ["open_space_roi_decider.cc"],
    hdrs = ["open_space_roi_decider.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/planning/common:planning_context",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/tasks/deciders:decider_base",
        "//modules/planning/proto:decision_proto",
        "//modules/routing/core",
    ],
)

cc_library(
    name = "open_space_pre_stop_decider",
    srcs = ["open_space_pre_stop_decider.cc"],
    hdrs = ["open_space_pre_stop_decider.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/planning/common:reference_line_info",
        "//modules/planning/common/util:common_lib",
        "//modules/planning/tasks/deciders:decider_base",
    ],
)

cc_library(
    name = "open_space_fallback_decider",
    srcs = ["open_space_fallback_decider.cc"],
    hdrs = ["open_space_fallback_decider.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/planning/common:planning_context",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/tasks/deciders:decider_base",
    ],
)


cpplint()
