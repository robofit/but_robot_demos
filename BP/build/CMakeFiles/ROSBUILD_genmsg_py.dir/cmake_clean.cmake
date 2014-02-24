FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/BP/msg"
  "../src/BP/srv"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/BP/msg/__init__.py"
  "../src/BP/msg/_PointOfInterest.py"
  "../src/BP/msg/_GoalCoords.py"
  "../src/BP/msg/_Detections.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
