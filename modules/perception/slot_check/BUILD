load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])


cc_library(
    name = "slot_shared_data",
    srcs = [
        "slot_shared_data.cc",
    ],
    hdrs = [
        "slot_shared_data.h",
    ],
    deps = [
        "//modules/perception/onboard:onboard",
      
        "//modules/perception/proto:slotcheck_Info_proto",
    ],
)

cc_library(
    name = "map_slot_check_lib",
    srcs = [
        "map_slotcheck_process_subnode.cc",
        "map_slotdet_process_subnode.cc",
    ],
    hdrs = [
        "map_slotcheck_process_subnode.h",
         "map_slotdet_process_subnode.h",
    ],
    deps = [
        "//modules/common",
        "//modules/common:log",
        "//modules/common/configs:config_gflags",
        "//modules/map/hdmap",
        "//modules/map/hdmap:hdmap_util",
        "//modules/perception/common",
        "//modules/localization/proto:localization_proto",
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/proto:sensor_proto",
        "//modules/perception/lib/base",
        "//modules/perception/onboard:onboard",
        "//modules/perception/proto:slotcheck_Info_proto",
        "//modules/perception/proto:perception_proto",
        "//modules/planning/proto:planning_proto",
        "@eigen",
        "//modules/perception/obstacle/camera/common",
        "//third_party/avm",
        "//modules/perception/obstacle/camera/detector/yolo_camera_detector:jmc_avm_ps_det",
    ],
)
cc_library(
    name = "slot_check_lib",
    srcs = [
       
         "slotcheck_process_subnode.cc",
            "slotdet_process_subnode.cc",        
            
           ],
    hdrs = [
        
            "slotcheck_process_subnode.h",
             "slotdet_process_subnode.h",         
             
           ],
    deps = [
        "//modules/common",
        "//modules/common:log",
        "//modules/common/configs:config_gflags",
        "//modules/map/hdmap",
        "//modules/map/hdmap:hdmap_util",
        "//modules/perception/common",
        "//modules/localization/proto:localization_proto",
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/proto:sensor_proto",
        "//modules/perception/lib/base",
        "//modules/perception/onboard:onboard",
        "//modules/perception/proto:slotcheck_Info_proto",
        "//modules/perception/proto:perception_proto",
        "//modules/control/proto:control_proto",
        "@eigen",
        "//modules/perception/obstacle/camera/common",
        "//third_party/avm",
        "//modules/perception/obstacle/camera/detector/yolo_camera_detector:jmc_avm_ps_det",
        "//modules/perception/slot_check:slot_shared_data",
    ],
)



cpplint()
