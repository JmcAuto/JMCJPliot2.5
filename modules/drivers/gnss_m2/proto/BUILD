package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "m2_proto",
    deps = [
        ":m2_proto_lib",
    ],
)

proto_library(
    name = "m2_proto_lib",
    srcs = [
        "m2.proto",
        "gnss_m2_conf.proto",
    ],
    deps = [
        "//modules/drivers/canbus/proto:canbus_proto_lib",
    ],
)
