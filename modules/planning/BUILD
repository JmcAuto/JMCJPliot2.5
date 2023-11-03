load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "planning_lib",
    srcs = ["planning.cc"],
    hdrs = ["planning.h"],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        ":on_lane_planning",
        "//modules/common/util:util",
        "//modules/common:jmc_auto_app",
        "//modules/common/util:message_util",
        "//modules/common/adapters:adapter_gflags",
        "//modules/localization/proto:localization_proto",
        "//modules/map/relative_map/proto:navigation_proto",
        "//modules/perception/proto:perception_proto",
        "//modules/planning/common:history",
        "//modules/planning/proto:planning_proto",
        "//modules/prediction/proto:prediction_proto",
        "@ros//:ros_common",
    ],
)

cc_binary(
    name = "planning",
    srcs = ["main.cc"],
    deps = [
        ":planning_lib",
        "//external:gflags",
    ],
)

cc_binary(
    name = "libplanning_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":planning_lib"],
)

cc_library(
    name = "planning_base",
    srcs = ["planning_base.cc"],
    hdrs = ["planning_base.h"],
    copts = [
        "-fopenmp",
    ],
    deps = [
        "//modules/common/util:util",
        "//modules/common:log",
        "//modules/common/configs:config_gflags",
        "//modules/common/math:quaternion",
        "//modules/common/proto:pnc_point_proto",
        "//modules/common/util:future",
        "//modules/common/util:message_util",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/localization/proto:localization_proto",
        "//modules/map/hdmap:hdmap_util",
        "//modules/perception/proto:perception_proto",
        "//modules/planning/common:history",
        "//modules/planning/common:planning_common",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:trajectory_stitcher",
        "//modules/planning/common/smoothers:smoother",
        "//modules/planning/common/util:util_lib",
        "//modules/planning/planner",
        "//modules/planning/planner:planner_dispatcher",
        "//modules/planning/proto:planning_proto",
        "//modules/planning/proto:traffic_rule_config_proto",
        "//modules/planning/tasks:task_factory",
        "//modules/planning/traffic_rules:traffic_decider",
        "//modules/prediction/proto:prediction_proto",
    ],
)


cc_library(
    name = "on_lane_planning",
    srcs = ["on_lane_planning.cc"],
    hdrs = ["on_lane_planning.h"],
    copts = [
        "-fopenmp",
    ],
    deps = [
        ":planning_base",
        "//modules/planning/common:planning_thread_pool",
    ],
)

filegroup(
    name = "planning_testdata",
    srcs = glob([
        "testdata/**",
    ]),
)

filegroup(
    name = "planning_conf",
    srcs = glob([
        "conf/**",
    ]),
)

cpplint()
