FILE(REMOVE_RECURSE
  "msg_gen"
  "src/grid_mapper/msg"
  "msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "src/grid_mapper/msg/__init__.py"
  "src/grid_mapper/msg/_NavigationFunction.py"
  "src/grid_mapper/msg/_OverlayClouds.py"
  "src/grid_mapper/msg/_LocalizedCloud.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
