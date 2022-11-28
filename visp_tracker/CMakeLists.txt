cmake_minimum_required(VERSION 3.5)
project(visp_tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)

# find_package(nodelet REQUIRED)
find_package(resource_retriever REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(image_proc REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(VISP REQUIRED COMPONENTS visp_core visp_vision visp_klt visp_mbt)

# messages and services interface generation
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/KltPoint.msg"
  "msg/KltPoints.msg"
  "msg/KltSettings.msg"
  "msg/MovingEdgeSettings.msg"
  "msg/MovingEdgeSite.msg"
  "msg/MovingEdgeSites.msg"
  "msg/TrackerSettings.msg"
  "srv/Init.srv"
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)

# get_cmake_property(_variableNames VARIABLES)
# list (SORT _variableNames)
# foreach (_variableName ${_variableNames})
# message(STATUS "${_variableName}=${${_variableName}}")
# endforeach()

# ???
# Dynamic reconfigure.
# generate_dynamic_reconfigure_options(
# cfg/ModelBasedSettings.cfg
# cfg/ModelBasedSettingsEdge.cfg
# cfg/ModelBasedSettingsKlt.cfg)

# catkin_package(
# LIBRARIES
# trackerNodelet
# ${PROJECT_NAME}

# CATKIN_DEPENDS
# dynamic_reconfigure
# geometry_msgs
# message_runtime
# nodelet
# roscpp
# sensor_msgs
# std_msgs
# tf

# DEPENDS
# VISP
# )

# include_directories(SYSTEM
# ${VISP_INCLUDE_DIRS}
# ${catkin_INCLUDE_DIRS}
# )

# include_directories(
# "${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}")

# ############
# Libraries #
# ############

# Library gathering libvisp_tracker used by all nodes.

# ament_export_dependencies(VISP)
add_library(${PROJECT_NAME}_common
  src/${PROJECT_NAME}/callbacks.cpp
  src/${PROJECT_NAME}/conversion.cpp
  src/${PROJECT_NAME}/file.cpp
  src/${PROJECT_NAME}/names.cpp
  src/${PROJECT_NAME}/tracker-client.cpp
  src/${PROJECT_NAME}/tracker-viewer.cpp
  src/${PROJECT_NAME}/tracker-mbt.cpp
)

ament_target_dependencies(${PROJECT_NAME}_common
  geometry_msgs
  image_transport
  resource_retriever
  rclcpp
  sensor_msgs
  std_msgs
  tf2_ros
  rclcpp
  image_proc
  VISP
)

# target_link_libraries(${PROJECT_NAME}_common
# ${VISP_LIBRARIES}
# "${cpp_typesupport_target}"
# )
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

target_include_directories(${PROJECT_NAME}_common PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_link_libraries(${PROJECT_NAME}_common
  ${VISP_LIBRARIES}
  "${cpp_typesupport_target}"
)

# #######
# Node
# #######
add_executable(${PROJECT_NAME}_mbt
  src/nodes/mbt.cpp
)

target_link_libraries(${PROJECT_NAME}_mbt
  ${PROJECT_NAME}_common
  "${cpp_typesupport_target}"
  ${VISP_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}_mbt
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
)

add_executable(${PROJECT_NAME}_client
  src/nodes/client.cpp
)

target_link_libraries(${PROJECT_NAME}_client
  ${PROJECT_NAME}_common
  "${cpp_typesupport_target}"
  ${VISP_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}_client
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
)

add_executable(${PROJECT_NAME}_viewer
  src/nodes/viewer.cpp
)

target_link_libraries(${PROJECT_NAME}_viewer
  ${PROJECT_NAME}_common
  "${cpp_typesupport_target}"
  ${VISP_LIBRARIES}
)

target_include_directories(${PROJECT_NAME}_common PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

# target_link_libraries(${PROJECT_NAME}_common
# ${VISP_LIBRARIES}
# "${cpp_typesupport_target}"
# )
install(
  TARGETS
  ${PROJECT_NAME}_viewer
  ${PROJECT_NAME}_client
  ${PROJECT_NAME}_mbt
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch models
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
