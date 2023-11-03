package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "remotecontrol_proto",
    deps = [
        ":remotecontrol_proto_lib",
    ],
)

proto_library(
    name = "remotecontrol_proto_lib",
    srcs = [
        
        "remote_control.proto",
        "remote_control_conf.proto",
    ],
    deps = [
        # "//modules/common/proto:common_proto_lib",
        # "//modules/common/proto:drive_state_proto_lib",
        "//modules/common/proto:header_proto_lib",
        # "//modules/common/proto:vehicle_signal_proto_lib",
        "//modules/canbus/proto:canbus_proto_lib",
    ],
)
