load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "fake_can_client",
    srcs = [
        "fake_can_client.cc",
    ],
    hdrs = [
        "fake_can_client.h",
    ],
    deps = [
        "//modules/drivers/canbus/can_client",
    ],
)

cpplint()
