package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "avm",
    srcs = [
        "lib/libAdjust.so",
        "lib/libLuaApi.so",
        "lib/libMath3D.so",
    ],
    hdrs = [
        "include/AdjustUtilities.h",
        "include/DefineCommon.h",
        "include/LuaContext.h",
        "include/Vector.h",
    ],
    #include_prefix = "esd_can",
)
