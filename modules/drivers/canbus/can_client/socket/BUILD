load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "socket_can_client_raw",
    srcs = [
        "socket_can_client_raw.cc",
    ],
    hdrs = [
        "socket_can_client_raw.h",
    ],
    deps = [
        "//modules/drivers/canbus/can_client",
    ],
)

cpplint()
