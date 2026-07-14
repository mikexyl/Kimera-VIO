cmake_minimum_required (VERSION 3.5)
project(kimera_vio VERSION 1.0 LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Fix for Clang 20 + Boost compatibility
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-error=non-template-argument-depends-on-template-parameter)
endif()

# Make sure that custom modules like FindXXXX are found
# Prefer to use system installation of gflags/glog
set(GFLAGS_PREFER_EXPORTED_GFLAGS_CMAKE_CONFIGURATION TRUE)
set(GLOG_PREFER_EXPORTED_GLOG_CMAKE_CONFIGURATION TRUE)
list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_SOURCE_DIR}/cmake)

IF(APPLE)
    # Fix linking on 10.14+. See https://stackoverflow.com/questions/54068035
    LINK_DIRECTORIES(/usr/local/lib)
ENDIF()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

message(STATUS "==============================================================")
message(STATUS "====================  Dependencies ===========================")

find_package(Gflags REQUIRED)
find_package(Glog 0.3.5 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(GTSAM_UNSTABLE REQUIRED)
find_package(opengv REQUIRED)
find_package(OpenCV REQUIRED)
find_package(DBoW2 REQUIRED)
find_package(KimeraRPGO REQUIRED)
find_package(gtsam_points REQUIRED)
# Pangolin is optional
find_package(Pangolin QUIET)
find_package(xfeat-cpp REQUIRED)
find_package(vio-factors REQUIRED)
find_package(vilib REQUIRED)

include(VerifyGtsamConfig)
option(KIMERA_VERIFY_GTSAM_CONFIG "Check that GTSAM was compiled with the right options" ON)
if (KIMERA_VERIFY_GTSAM_CONFIG)
    verify_gtsam_config()
endif()

### External Dependencies
include(ExternalProject)

### Download and unzip the vocabularly file
if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/vocabulary/ORBvoc.yml)
  message(STATUS "Downloading vocabulary file from dropbox.")
  file(DOWNLOAD
       http://www.dropbox.com/s/lyo0qgbdxn6eg6o/ORBvoc.zip?dl=1
       ${CMAKE_CURRENT_SOURCE_DIR}/vocabulary/ORBvoc.zip
       SHOW_PROGRESS
       STATUS orbvoc_download_success
       TIMEOUT 60)
  if(orbvoc_download_success)
    message(STATUS "Unzipping vocabulary file.")

    execute_process(COMMAND ${CMAKE_COMMAND} -E tar xzf ${CMAKE_CURRENT_SOURCE_DIR}/vocabulary/ORBvoc.zip
                            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/vocabulary/)
    message(STATUS "Moving vocabulary file.")
    execute_process(COMMAND ${CMAKE_COMMAND} -E copy_directory
                    ${CMAKE_CURRENT_SOURCE_DIR}/vocabulary/ORBvoc/
                    ${CMAKE_CURRENT_SOURCE_DIR}/vocabulary/)
    execute_process(COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_CURRENT_SOURCE_DIR}/vocabulary/ORBvoc/)
  else(orbvoc_download_success)
    message(STATUS "Failed to download vocabulary file. Please download manually.")
  endif(orbvoc_download_success)
else()
  message(STATUS "Vocabulary file exists, will not download.")
endif()

### Handle third party libraries
add_subdirectory(third_party)

### Compile the code
add_library(${PROJECT_NAME} SHARED "")

### Add source code for data provider.
add_subdirectory(include/kimera-vio/dataprovider)
add_subdirectory(src/dataprovider)
### Add source code for data provider.
add_subdirectory(include/kimera-vio/playground)
add_subdirectory(src/playground)
### Add source code for Frontend.
add_subdirectory(include/kimera-vio/frontend)
add_subdirectory(src/frontend)
### Add source code for Backend.
add_subdirectory(include/kimera-vio/backend)
add_subdirectory(src/backend)
### Add source code for factors
add_subdirectory(include/kimera-vio/factors)
add_subdirectory(src/factors)
### Add source code for mesh
add_subdirectory(include/kimera-vio/mesh)
add_subdirectory(src/mesh)
### Add source code for initialization
add_subdirectory(include/kimera-vio/initial)
add_subdirectory(src/initial)
### Add source code for utils
add_subdirectory(include/kimera-vio/utils)
add_subdirectory(src/utils)
### Add source code for pipeline
add_subdirectory(include/kimera-vio/pipeline)
add_subdirectory(src/pipeline)
### Add source code for common
add_subdirectory(include/kimera-vio/common)
add_subdirectory(src/common)
### Add source code for loopclosure
add_subdirectory(include/kimera-vio/loopclosure)
add_subdirectory(src/loopclosure)
### Add source code for logging
add_subdirectory(include/kimera-vio/logging)
add_subdirectory(src/logging)
### Add source code for imu Frontend
add_subdirectory(include/kimera-vio/imu-frontend)
add_subdirectory(src/imu-frontend)
### Add source code for visualizer.
add_subdirectory(include/kimera-vio/visualizer)
add_subdirectory(src/visualizer)

target_link_libraries(${PROJECT_NAME}
  PRIVATE
    KimeraRPGO
    triangle::triangle
  PUBLIC
    ${OpenCV_LIBRARIES}
    ${GFLAGS_LIBRARIES}
    ${GLOG_LIBRARIES}
    ${DBoW2_LIBRARIES}
    gtsam
    gtsam_unstable
    gtsam_points::gtsam_points
    opengv
    xfeat-cpp::xfeat-cpp
    xfeat-cpp::sgm
    vio-factors
    vilib::vilib
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    ${DBoW2_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${GFLAGS_INCLUDE_DIRS}
    ${GLOG_INCLUDE_DIRS}
    ${GTSAM_INCLUDE_DIR}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Pangolin is optional
if(Pangolin_FOUND)
  target_link_libraries(${PROJECT_NAME}
    PUBLIC
      ${Pangolin_LIBRARIES}
  )
  target_include_directories(${PROJECT_NAME}
    PUBLIC
      ${Pangolin_INCLUDE_DIRS}
  )
  target_compile_definitions(${PROJECT_NAME} PRIVATE Pangolin_FOUND=1)
else(Pangolin_FOUND)
  message(STATUS "Pangolin not found.")
endif(Pangolin_FOUND)

target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wno-unused-parameter -pipe)

# We would just need to say cxx_std_11 if we were using cmake 3.8
target_compile_features(${PROJECT_NAME} PUBLIC
        cxx_auto_type cxx_constexpr cxx_range_for cxx_nullptr cxx_override)

# Add an alias so that library can be used inside the build tree,
# e.g. when testing
add_library(kimera_vio::kimera_vio ALIAS kimera_vio)

option(KIMERA_BUILD_EXAMPLES "Build Kimera-VIO examples" ON)
if(KIMERA_BUILD_EXAMPLES)
  add_executable(stereoVIOEuroc ./examples/KimeraVIO.cpp)
  target_link_libraries(stereoVIOEuroc PUBLIC kimera_vio::kimera_vio)
endif()


############################### TESTS ##########################################
option(KIMERA_BUILD_TESTS "Build Kimera-VIO tests" ON)
if(KIMERA_BUILD_TESTS)
  enable_testing()
  add_executable(testEdgeSelection tests/testEdgeSelection.cpp)
  target_link_libraries(testEdgeSelection PUBLIC kimera_vio::kimera_vio)

  add_executable(testMonoDepthConfidence tests/testMonoDepthConfidence.cpp)
  target_link_libraries(testMonoDepthConfidence PUBLIC kimera_vio::kimera_vio)
  add_test(NAME testMonoDepthConfidence COMMAND testMonoDepthConfidence)

  add_executable(testDa3KeyframeSelector tests/testDa3KeyframeSelector.cpp)
  target_link_libraries(testDa3KeyframeSelector PUBLIC kimera_vio::kimera_vio)
  add_test(NAME testDa3KeyframeSelector COMMAND testDa3KeyframeSelector)

  add_executable(testMonoDepthScaleAlignment
    tests/testMonoDepthScaleAlignment.cpp)
  target_link_libraries(testMonoDepthScaleAlignment
    PUBLIC kimera_vio::kimera_vio)
  add_test(NAME testMonoDepthScaleAlignment
    COMMAND testMonoDepthScaleAlignment)

  add_executable(testCameraAwareEssentialMatrixFactor
    tests/testCameraAwareEssentialMatrixFactor.cpp)
  target_link_libraries(testCameraAwareEssentialMatrixFactor
    PUBLIC kimera_vio::kimera_vio)
  add_test(NAME testCameraAwareEssentialMatrixFactor
    COMMAND testCameraAwareEssentialMatrixFactor)

  add_executable(testDa3EssentialMatrixFactors
    tests/testDa3EssentialMatrixFactors.cpp)
  target_link_libraries(testDa3EssentialMatrixFactors
    PUBLIC kimera_vio::kimera_vio)
  add_test(NAME testDa3EssentialMatrixFactors
    COMMAND testDa3EssentialMatrixFactors)
endif()

############################### INSTALL/EXPORT #################################
## We install the export that we defined above
## Export the targets to a script
## This will install the import script kimera_vioTargets.cmake
## When findpackaged by other scripts, it will load the targets defined
## in the export kimera_vio-export.
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/kimera_vioConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion
)

# Create kimera_vioConfig.cmake with extra info from kimera_vioConfig.cmake.in
# This file is necessary to find_package the library kimera_vio.
set(INSTALL_CONFIGDIR lib/cmake/kimera_vio)
configure_package_config_file(
  ${CMAKE_CURRENT_LIST_DIR}/cmake/kimera_vioConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/kimera_vioConfig.cmake
  INSTALL_DESTINATION ${INSTALL_CONFIGDIR}
)

option(EXPORT_KIMERA "Export kimera_vio instead of installing it." OFF)
if(EXPORT_KIMERA)
  export(TARGETS ${PROJECT_NAME}
         FILE kimera_vioTargets.cmake)
       export(PACKAGE ${PROJECT_NAME})
else(EXPORT_KIMERA)
  include(GNUInstallDirs)
  ## First of all, we need to install the library itself.
  install(TARGETS ${PROJECT_NAME}
      EXPORT kimera_vio-export
      LIBRARY DESTINATION lib
      ARCHIVE DESTINATION lib
      # Perhaps use this, instead of installing include dir
      #PUBLIC_HEADER DESTINATION include
      INCLUDES DESTINATION include # We need this right?
      RUNTIME DESTINATION bin
  )
  install(EXPORT kimera_vio-export
    FILE
      kimera_vioTargets.cmake
    DESTINATION
      ${INSTALL_CONFIGDIR}
  )
  # Install header files
  install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/include/
    DESTINATION include
    FILES_MATCHING PATTERN "*.h")

  ## Install the config and configversion
  install(FILES
      ${CMAKE_CURRENT_BINARY_DIR}/kimera_vioConfig.cmake
      ${CMAKE_CURRENT_BINARY_DIR}/kimera_vioConfigVersion.cmake
      ${CMAKE_CURRENT_LIST_DIR}/cmake/FindGflags.cmake
      ${CMAKE_CURRENT_LIST_DIR}/cmake/FindGlog.cmake
      DESTINATION ${INSTALL_CONFIGDIR}
  )
endif(EXPORT_KIMERA)

################################################################################
# Print configuration variables
message(STATUS "===============================================================")
message(STATUS "================  Configuration Options  ======================")
message(STATUS "CMAKE_CXX_COMPILER_ID type                : ${CMAKE_CXX_COMPILER_ID}")
message(STATUS "CMAKE_CXX_COMPILER_VERSION                : ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "Build flags                                               ")
if(NOT MSVC AND NOT XCODE_VERSION)
  message(STATUS "  Build type                              : ${CMAKE_BUILD_TYPE}")
  if (NOT (${CMAKE_BUILD_TYPE} STREQUAL "Release" OR ${CMAKE_BUILD_TYPE} STREQUAL "RelWithDebInfo"))
    message(WARNING "  Build type is not in RELEASE, this will slow down the code.")
  endif()
  message(STATUS "  C compilation flags (Release)           : ${CMAKE_C_FLAGS} ${CMAKE_C_FLAGS_RELEASE}")
  message(STATUS "  C++ compilation flags (Release)         : ${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_RELEASE}")
endif()
