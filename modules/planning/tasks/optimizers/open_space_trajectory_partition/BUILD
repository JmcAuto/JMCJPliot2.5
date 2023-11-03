load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "open_space_trajectory_partition",
    srcs = ["open_space_trajectory_partition.cc"],
    hdrs = ["open_space_trajectory_partition.h"],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        "//modules/common/status",
        "//modules/planning/proto:planning_config_proto",
        "//modules/planning/tasks:task",
        "//modules/planning/tasks/optimizers:trajectory_optimizer",
        "//modules/common/time:time",
        "@com_google_absl//absl/strings",
    ],
)


cpplint()
