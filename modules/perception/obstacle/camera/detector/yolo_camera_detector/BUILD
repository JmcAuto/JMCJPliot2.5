load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "util",
    srcs = ["util.cc"],
    hdrs = ["util.h"],
    deps = [
        "//modules/common:log",
        "//modules/perception/obstacle/base",
        "//modules/perception/obstacle/camera/common",
        "@caffe//:lib",
        "@eigen",
        "@opencv2//:core",
    ],
)

cc_library(
    name = "jmc_caffe_detector",
    srcs = [
            "jmc_caffe_detector.cc",
            "jmc_object_detector.cc"
           ],
    hdrs = [
            "jmc_caffe_detector.h",
            "jmc_object_detector.h",
	    "jmc_objdet_3d.h"
           ],
    deps = [
        ":util",
        "//modules/common:log",
        "//modules/perception/common",
        "//modules/perception/cuda_util",
        "//modules/perception/lib/base",
        "//modules/perception/lib/config_manager",
        "//modules/perception/obstacle/base",
        "//modules/perception/obstacle/camera/common:cnn_adapter",
        "//modules/perception/obstacle/camera/detector/common:feature_extractor",
        "//modules/perception/obstacle/camera/detector/common/proto:tracking_feature_proto",
        "//modules/perception/obstacle/camera/detector/yolo_camera_detector/proto:yolo_camera_proto",
        "//modules/perception/obstacle/camera/dummy",
        "//modules/perception/obstacle/camera/interface",
        "//modules/perception/proto:yolo_camera_detector_config_lib_proto",
        "@caffe//:lib",
        "@eigen",
    ],
)

cc_library(
    name = "jmc_avm_ps_det",
    srcs = [
            "jmc_psdet.cc",
            "jmc_avm.cc",
            ],
    hdrs = [
            "jmc_psdet.h",
            "jmc_avm.h",
            ],
    deps = [
        ":util",
        "//modules/common:log",
        "//modules/perception/common",
        "//modules/perception/cuda_util",
        "//modules/perception/lib/base",
        "//modules/perception/lib/config_manager",
        "//modules/perception/obstacle/base",
        "//modules/perception/obstacle/camera/common:cnn_adapter",
        "//modules/perception/obstacle/camera/detector/common:feature_extractor",
        "//modules/perception/obstacle/camera/detector/common/proto:tracking_feature_proto",
        "//modules/perception/obstacle/camera/detector/yolo_camera_detector/proto:yolo_camera_proto",
        "//modules/perception/obstacle/camera/dummy",
        "//modules/perception/obstacle/camera/interface",
        "//modules/perception/proto:yolo_camera_detector_config_lib_proto",
        "@caffe//:lib",
        "@eigen",
        "//third_party/avm",
    ],
)

cpplint()
