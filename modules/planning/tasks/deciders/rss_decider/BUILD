load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "rss_decider",
    srcs = ["rss_decider.cc"],
    hdrs = ["rss_decider.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/common/status",
        "//modules/planning/common:planning_context",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/proto:planning_proto",
        "//modules/planning/tasks:task",
        "@ad_rss_lib//:ad_rss",
    ],
)


cpplint()
