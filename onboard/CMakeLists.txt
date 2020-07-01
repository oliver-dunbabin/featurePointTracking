cmake_minimum_required(VERSION 3.10)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(THREADS_PREFER_PTHREAD_FLAG ON)

# set project name
project(featureTracking)

# set RealSense library and directory path
set( RSLIBPATH "/home/oliver/MySoftware/RealSense_T265/build-librealsense-Desktop-Default/")
set( RSDIRPATH "/home/oliver/MySoftware/RealSense_T265/librealsense/include/librealsense2")

# set Vicon datastream library path
set( VICONLIBPATH "/home/oliver/ViconDataStreamSDK_1.7/Linux64-boost-1.58.0/20170125_96542h.x64")

# set gnuplot-iostream library path
set( GNULIBPATH "/home/oliver/gnuplot-iostream/")

# set location of library and executable
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})

# find realsense2 library
find_library(REALSENSE2_FOUND realsense2 HINTS ${RSLIBPATH} REQUIRED)
if(NOT REALSENSE2_FOUND)
    set(REALSENSE2_FOUND "realsense2")
    message(WARN "failed to find library(realsense2)")
endif()

# find opencv package
find_package(OpenCV REQUIRED)

# find pthread package
find_package(Threads REQUIRED)

# find Vicon SDK library
find_library(VICONDATASTREAMSDK_CPP_FOUND ViconDataStreamSDK_CPP HINTS ${VICONLIBPATH} REQUIRED)

# find Boost packages
find_package(Boost 1.58.0 REQUIRED COMPONENTS system iostreams)

# Include directories of source and headers
include_directories(
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/src
        ${RSDIRPATH}
        ${VICONLIBPATH}
        ${GNULIBPATH}
)

# Adds all files in subdirectories
file(GLOB all_SRCS
        "${PROJECT_SOURCE_DIR}/include/*.h"
        "${PROJECT_SOURCE_DIR}/include/*.hpp"
        "${PROJECT_SOURCE_DIR}/src/*.cpp"
        "${PROJECT_SOURCE_DIR}/src/*.c")

# add the executable
add_executable(${PROJECT_NAME} ${all_SRCS})

# link with libraries
target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenCV_LIBS} ${REALSENSE2_FOUND} ${VICONDATASTREAMSDK_CPP_FOUND} ${Boost_LIBRARIES} Threads::Threads)
