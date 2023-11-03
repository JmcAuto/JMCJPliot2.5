load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "vins_common",
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
    copts = ["-std=c99"],
)


cpplint()