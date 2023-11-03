load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "planner",
    hdrs = ["planner.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/common/proto:pnc_point_proto",
        "//modules/common/status",
        "//modules/planning/common:frame",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/proto:planning_proto",
        "//modules/planning/scenarios:scenario",
        "//modules/planning/scenarios:scenario_manager",
    ],
)

cc_library(
    name = "planner_dispatcher",
    srcs = [
        "on_lane_planner_dispatcher.cc",
        "planner_dispatcher.cc",
    ],
    hdrs = [
        "on_lane_planner_dispatcher.h",
        "planner_dispatcher.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
        "-fopenmp",
    ],
    deps = [
        "//modules/common/status",
        "//modules/common/util",
        "//modules/planning/planner/public_road:public_road_planner",
        "//modules/planning/proto:planning_proto",
    ],
)


cpplint()
