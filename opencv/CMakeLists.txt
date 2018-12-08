cmake_minimum_required(VERSION 3.4.1)
set(OpenCV_DIR "src/sdk/native/jni")
find_package(OpenCV REQUIRED)
message(STATUS "OpenCV libraries: ${OpenCV_LIBS}")
include_directories(${OpenCV_INCLUDE_DIRS})

set( CMAKE_CXX_FLAGS "-std=c++11" )
#target_link_libraries( ${OpenCV_LIBS} )