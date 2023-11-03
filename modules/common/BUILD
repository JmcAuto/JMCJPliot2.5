load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "common",
    deps = [
        ":log",
        ":macro",
        "@ros//:ros_common",
    ],
)

cc_library(
    name = "macro",
    hdrs = [
        "macro.h",
    ],
)

cc_library(
    name = "macros",
    hdrs = [
        "macros.h",
    ],
)

cc_library(
    name = "log",
    hdrs = [
        "log.h",
    ],
    deps = [
        "@glog//:glog",
    ],
)

cc_library(
    name = "jmc_auto_app",
    srcs = [
        "jmc_auto_app.cc",
    ],
    hdrs = [
        "jmc_auto_app.h",
    ],
    deps = [
        ":log",
        "//modules/common/status",
        "//modules/common/util:string_util",
        "@ros//:ros_common",
    ],
)

cpplint()
