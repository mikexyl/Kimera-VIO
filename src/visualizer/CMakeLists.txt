### Add source code
target_sources(kimera_vio
  PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/Visualizer3D.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/Visualizer3DModule.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/Visualizer3DFactory.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/OpenCvDisplayParams.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/DisplayParams.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/Display.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/DisplayModule.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/DisplayFactory.cpp"
)

if(KIMERA_BUILD_OPENCV_VIZ)
  target_sources(kimera_vio PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/OpenCvVisualizer3D.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/OpenCvDisplay.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/Display-definitions.cpp")
endif()

if(Pangolin_FOUND)
  target_sources(kimera_vio PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/PangolinDisplay.cpp"
  )
endif(Pangolin_FOUND)
