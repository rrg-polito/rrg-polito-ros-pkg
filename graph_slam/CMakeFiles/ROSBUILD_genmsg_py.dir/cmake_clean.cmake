FILE(REMOVE_RECURSE
  "msg_gen"
  "src/graph_slam/msg"
  "msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "src/graph_slam/msg/__init__.py"
  "src/graph_slam/msg/_LocalizedCloud.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
