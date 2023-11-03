load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "rtk_m2_localization",
    srcs = [
        "rtk_localization.cc",
    ],
    hdrs = [
        "rtk_localization.h",
    ],
    # linkopts = [
    #     "-lz",
    #     "-lopencv_core",
    #     "-lopencv_highgui",
    #     "-lopencv_imgproc",
    # ],
    deps = [
        "//modules/common",
        "//modules/common/adapters:adapter_manager",
        "//modules/common/monitor_log",
        "//modules/common/proto:common_proto",
        "//modules/common/status",
        "//modules/common/time",
        "//modules/localization:localization_base",
        "//modules/localization/common:localization_common",
        "//modules/localization/proto:localization_config_proto",
        "//modules/localization/proto:localization_proto",
        "@gtest",
        "@ros//:ros_common",

        "//modules/canbus/proto:canbus_proto",
        "@proj4//:proj4",

    ],
)


cpplint()
