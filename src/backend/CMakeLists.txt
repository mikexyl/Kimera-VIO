### Add source code
target_sources(kimera_vio PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/DenseMap.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/MonoDepthAlignment.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/MonoDepthVGICPFactors.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/VioBackendModule.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/VioBackend.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/VioBackendParams.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/RegularVioBackend.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/RegularVioBackendParams.cpp"
)
