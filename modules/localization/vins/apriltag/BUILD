load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "vins_apriltag",
    # srcs = [
    #     "homography.c",
    #     "image_u8x3.c",
    #     "matd.c",
    #     "zarray.c",
    #     "svd22.c",
    #     "pnm.c",
    # ],
    srcs = glob(["*.c"]),
    hdrs = glob(["*.h"]),
    # hdrs = [
    #     "homography.h",
    #     "image_u8x3.h",
    #     "matd.h",
    #     "zarray.h",
    #     "svd22.h",
    #     "pnm.h",

    # ],
    deps = [
        "//modules/localization/vins/apriltag/common:vins_common",
    ],
    copts = ["-std=c99"],
)


cpplint()