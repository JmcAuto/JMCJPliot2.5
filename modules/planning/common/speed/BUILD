load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "st_point",
    srcs = ["st_point.cc"],
    hdrs = ["st_point.h"],
    deps = [
        "//modules/common/math",
        "//external:gflags",
    ],
)

cc_library(
    name = "st_boundary",
    srcs = ["st_boundary.cc"],
    hdrs = ["st_boundary.h"],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        ":st_point",
        "//modules/common:log",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/proto:planning_proto",
    ],
)


cc_library(
    name = "speed_data",
    srcs = ["speed_data.cc"],
    hdrs = ["speed_data.h"],
    deps = [
        ":st_point",
        "//modules/common/math",
        "//modules/common/proto:pnc_point_proto",
        "//modules/common/util:point_factory",
        "//modules/common/util:string_util",
        "//modules/planning/common:planning_context",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/proto:planning_proto",
        "//external:gflags",
    ],
)

cpplint()
