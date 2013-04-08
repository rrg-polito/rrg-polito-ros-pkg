FILE(REMOVE_RECURSE
  "msg_gen"
  "src/grid_mapper/msg"
  "msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "msg_gen/cpp/include/grid_mapper/NavigationFunction.h"
  "msg_gen/cpp/include/grid_mapper/OverlayClouds.h"
  "msg_gen/cpp/include/grid_mapper/LocalizedCloud.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
