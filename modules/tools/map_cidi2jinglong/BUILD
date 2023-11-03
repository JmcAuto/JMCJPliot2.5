load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "map_cidi2jinglong",
    srcs = ["map_cidi2jinglong.cc"],
    data = ["//modules/map:map_data"],
    deps = [
        "//external:gflags",
        "//modules/common",
        "//modules/common/configs:config_gflags",
        "//modules/common/util",
        "//modules/common/util:points_downsampler",
        "//modules/map/hdmap:hdmap_util",
        # "//modules/map/hdmap/adapter:opendrive_adapter",
        "//modules/tools/map_cidi2jinglong/jinglong_proto:jinglong_map_proto",
        "//modules/tools/map_cidi2jinglong/cidi_proto:cidi_map_proto",
        "//modules/tools/map_cidi2jinglong/topo_creator:graph_creator",
        "//modules/routing/common:routing_gflags",
        "//modules/routing/proto:routing_proto",
    ],
)


cpplint()
