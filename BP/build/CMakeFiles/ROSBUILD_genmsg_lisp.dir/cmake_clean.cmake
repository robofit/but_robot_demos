FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/BP/msg"
  "../src/BP/srv"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "../msg_gen/lisp/PointOfInterest.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_PointOfInterest.lisp"
  "../msg_gen/lisp/GoalCoords.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_GoalCoords.lisp"
  "../msg_gen/lisp/Detections.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_Detections.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
