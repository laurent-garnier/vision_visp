cmake_minimum_required(VERSION 3.5)
project(visp_auto_tracker)

# Since visp 3.5.0 to avoid the following warning that makes build unstable
#
# /usr/include/OGRE/OgreMemorySTLAllocator.h:130:23: warning: ISO C++17 does not allow
#   ‘register’ storage class specifier [-Wregister]
#
# we try to add -Wno-register option
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-Wno-register" COMPILER_SUPPORTS_W_NO_REGISTER)

if(COMPILER_SUPPORTS_W_NO_REGISTER)
  set(EXTRA_CXX_FLAGS "-Wno-register")
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(geometry_msgs REQUIRED) 
find_package(message_filters REQUIRED )
find_package(resource_retriever REQUIRED) 
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED) 
find_package(std_msgs REQUIRED)
find_package(visp_bridge REQUIRED) 
find_package(visp_tracker REQUIRED) 

find_package(VISP REQUIRED COMPONENTS visp_core)

if(VISP_HAVE_ZBAR OR VISP_HAVE_DMTX OR VISP_HAVE_APRILTAG)


include_directories(
  ${VISP_INCLUDE_DIRS}
)

# catkin_package(
#   LIBRARIES auto_tracker

#   CATKIN_DEPENDS
#     geometry_msgs
#     message_filters
#     roscpp
#     sensor_msgs
#     std_msgs
#     visp_tracker

#   DEPENDS
#     VISP
# )

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/flashcode_mbt)

# Library gathering libauto_tracker used by all nodes.
add_library(${PROJECT_NAME}_cmd_line
  flashcode_mbt/cmd_line/cmd_line.cpp
  flashcode_mbt/cmd_line/cmd_line.h
)
target_link_libraries(${PROJECT_NAME}_cmd_line
  ${resource_retriever_LIBRARIES}
)

#catkin_lint: ignore unquoted_string_op
if(VISP_VERSION VERSION_LESS "2.10.0")
  add_library(${PROJECT_NAME}_qrcode_detector
    flashcode_mbt/detectors/detector_base.cpp
    flashcode_mbt/detectors/detector_base.h
    flashcode_mbt/detectors/qrcode/detector.cpp
    flashcode_mbt/detectors/qrcode/detector.h
  )
  target_link_libraries(${PROJECT_NAME}_qrcode_detector
    ${resource_retriever_LIBRARIES}
    zbar)

  add_library(${PROJECT_NAME}_datamatrix_detector
    flashcode_mbt/detectors/datamatrix/detector.cpp
    flashcode_mbt/detectors/datamatrix/detector.h
    flashcode_mbt/detectors/detector_base.cpp
    flashcode_mbt/detectors/detector_base.h
  )
  target_link_libraries(${PROJECT_NAME}_datamatrix_detector
    ${resource_retriever_LIBRARIES}
    dmtx)
endif()

add_library(auto_tracker
  flashcode_mbt/libauto_tracker/events.h
  flashcode_mbt/libauto_tracker/logfilewriter.hpp
  flashcode_mbt/libauto_tracker/states.hpp
  flashcode_mbt/libauto_tracker/threading.cpp
  flashcode_mbt/libauto_tracker/threading.h
  flashcode_mbt/libauto_tracker/tracking.cpp
  flashcode_mbt/libauto_tracker/tracking.h)

#catkin_lint: ignore unquoted_string_op
if(VISP_VERSION VERSION_LESS "2.10.0")
  target_link_libraries(auto_tracker
      ${PROJECT_NAME}_cmd_line
      ${PROJECT_NAME}_qrcode_detector
      ${PROJECT_NAME}_datamatrix_detector
      ${resource_retriever_LIBRARIES}
  )
else()
  target_link_libraries(auto_tracker
      ${PROJECT_NAME}_cmd_line
      ${resource_retriever_LIBRARIES}
  )
endif()

add_executable(${PROJECT_NAME}
  src/main.cpp
  src/names.cpp
  src/names.h
  src/node.cpp
  src/node.h
  )

if(EXTRA_CXX_FLAGS)
    set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS ${EXTRA_CXX_FLAGS})
    set_target_properties(${PROJECT_NAME}_cmd_line PROPERTIES COMPILE_FLAGS ${EXTRA_CXX_FLAGS})
    set_target_properties(auto_tracker PROPERTIES COMPILE_FLAGS ${EXTRA_CXX_FLAGS})
endif()

if(COLCON_ENABLE_TESTING)
  add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_bag)
endif()

#catkin_lint: ignore unquoted_string_op
if(VISP_VERSION VERSION_LESS "2.10.0")
  target_link_libraries(${PROJECT_NAME}
    auto_tracker
    ${PROJECT_NAME}_qrcode_detector
    ${PROJECT_NAME}_datamatrix_detector
    ${PROJECT_NAME}_cmd_line
    dmtx zbar)

  target_link_libraries(${PROJECT_NAME}
    ${resource_retriever_LIBRARIES})

  install(
    TARGETS
      auto_tracker
      ${PROJECT_NAME}
      ${PROJECT_NAME}_cmd_line
      ${PROJECT_NAME}_datamatrix_detector
      ${PROJECT_NAME}_qrcode_detector
    ARCHIVE DESTINATION lib/${PROJECT_NAME}
    LIBRARY DESTINATION lib/${PROJECT_NAME}
    RUNTIME DESTINATION bin/${PROJECT_NAME}
  )
else()
  target_link_libraries(${PROJECT_NAME}
    auto_tracker
    ${resource_retriever_LIBRARIES})

  install(
    TARGETS
      auto_tracker
      ${PROJECT_NAME}
      ${PROJECT_NAME}_cmd_line
    ARCHIVE DESTINATION lib/${PROJECT_NAME}
    LIBRARY DESTINATION lib/${PROJECT_NAME}
    RUNTIME DESTINATION bin/${PROJECT_NAME}
  )
endif()

install(DIRECTORY launch
  DESTINATION ${PROJECT_NAME}
)

install(DIRECTORY models
  DESTINATION ${PROJECT_NAME}
)

#############
# Bag files #
#############

# Tutorial
if(COLCON_ENABLE_TESTING)
  catkin_download_test_data(
    ${PROJECT_NAME}_bag
    https://github.com/lagadic/vision_visp/releases/download/vision_visp-0.5.0/tutorial-qrcode.bag
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/bag
    MD5 0f80ceea2610b8400591ca7aff764dfa)

  install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/bag
    DESTINATION ${PROJECT_NAME}
  )
endif()
else() # VISP_HAVE_ZBAR OR VISP_HAVE_DMTX OR VISP_HAVE_APRILTAG
message("${PROJECT_NAME} build is disabled since ViSP is not built with libzbar-dev 3rd party")
endif() # VISP_HAVE_ZBAR OR VISP_HAVE_DMTX OR VISP_HAVE_APRILTAG

