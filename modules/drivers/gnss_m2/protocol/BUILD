load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "canbus_m2_protocol",
    srcs = glob([
        "*.cc",
    ]),
    hdrs = glob([
        "*.h",
    ]),
    deps = [
        "//modules/drivers/canbus/common:canbus_common",
        "//modules/drivers/gnss_m2/proto:m2_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
    ],
)

cpplint()
