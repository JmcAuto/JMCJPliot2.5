load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "modest_detector",
    srcs = [
        "conti_radar_id_expansion.cc",
        "conti_radar_util.cc",
        "modest_radar_detector.cc",
        "object_builder.cc",
        "radar_track.cc",
        "radar_track_manager.cc",
        "radar_util.cc",
    ],
    hdrs = [
        "conti_radar_id_expansion.h",
        "conti_radar_util.h",
        "modest_radar_detector.h",
        "object_builder.h",
        "radar_define.h",
        "radar_track.h",
        "radar_track_manager.h",
        "radar_util.h",
    ],
    deps = [
        "//modules/common:log",
        "//modules/perception/lib/config_manager",
        "//modules/perception/obstacle/common",
        "//modules/perception/obstacle/radar/interface",
        "//modules/perception/proto:modest_radar_detector_config_lib_proto",
        "@eigen",
        "@pcl",
    ],
)

cpplint()
