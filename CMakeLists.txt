cmake_minimum_required(VERSION 2.8)

project(Kinect2CameraInterface)

set(CMAKE_BUILD_TYPE Release)

set(HOME ${CMAKE_CURRENT_SOURCE_DIR})
set(CMAKE_INSTALL_PREFIX ${HOME})

set(KINECT_SDK_PATH "C:/Program Files/Microsoft SDKs/Kinect/v2.0-PublicPreview1408")


include_directories(${HOME}/include)
include_directories(${KINECT_SDK_PATH}/inc)

link_directories(${KINECT_SDK_PATH}/Lib/x86)

set(KINECT2_LIBS Kinect20)

add_subdirectory(${HOME}/src)
