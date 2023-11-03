load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "valet_parking",
    srcs = [
        "stage_approaching_parking_spot.cc",
        "stage_parking.cc",
        "valet_parking_scenario.cc",
    ],
    hdrs = [
        "stage_approaching_parking_spot.h",
        "stage_parking.h",
        "valet_parking_scenario.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/common:log",
        "//modules/planning/proto:planning_proto",
        "//modules/planning/scenarios:scenario",
    ],
)

cpplint()
