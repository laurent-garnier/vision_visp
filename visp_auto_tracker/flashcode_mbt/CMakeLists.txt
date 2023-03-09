cmake_minimum_required(VERSION 3.0.0)

PROJECT( tracking )
FIND_PACKAGE( OpenCV REQUIRED )
FIND_PACKAGE(VISP REQUIRED)
IF(VISP_FOUND)
  INCLUDE(${VISP_USE_FILE})
ENDIF(VISP_FOUND)

INCLUDE_DIRECTORIES(${OpenCV_DIR})


add_subdirectory(cmd_line)
add_subdirectory(detectors)
include_directories(detectors)
include_directories(auto_tracker)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
add_library(${PROJECT_NAME}_auto_tracker 
			libauto_tracker/states.hpp 
			libauto_tracker/events.h 
			libauto_tracker/tracking.h 
			libauto_tracker/tracking.cpp 
			libauto_tracker/logfilewriter.hpp 
			libauto_tracker/threading.h 
			libauto_tracker/threading.cpp)

ament_target_dependencies(${PROJECT_NAME}_auto_tracker
			rclcpp
			VISP)
		  
ADD_EXECUTABLE( tracking examples/complex.cpp )
TARGET_LINK_LIBRARIES( tracking auto_tracker ${OpenCV_LIBS} qrcode_detector datamatrix_detector dmtx zbar cmd_line)

ADD_EXECUTABLE( tracking_simple examples/simple.cpp )
TARGET_LINK_LIBRARIES( tracking_simple auto_tracker ${OpenCV_LIBS} qrcode_detector datamatrix_detector dmtx zbar cmd_line )
