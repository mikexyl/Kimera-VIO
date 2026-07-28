### Add source code
if(KIMERA_BUILD_OPENCV_VIZ)
  target_sources(kimera_vio
      PRIVATE
      "${CMAKE_CURRENT_LIST_DIR}/EurocPlayground.cpp"
  )
endif()
