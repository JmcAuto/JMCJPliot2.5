bazel-bin/modules/tools/map_cidi2jinglong/map_cidi2jinglong -map_dir=modules/map/data/cidi_map -output_dir=modules/map/data/cidi_map/
bash scripts/generate_routing_topo_graph.sh -map_dir=modules/map/data/cidi_map/
bazel-bin/modules/map/tools/sim_map_generator -map_dir=modules/map/data/cidi_map -output_dir=modules/map/data/cidi_map/


