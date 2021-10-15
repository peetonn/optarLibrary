cmake_minimum_required(VERSION 3.10)

set(LIBRARY_NAME optar)
project(${LIBRARY_NAME} VERSION 1.0.0)

set(SOURCES optar.cpp ${CMAKE_SOURCE_DIR}/include/optar.hpp
            ${CMAKE_SOURCE_DIR}/include/config.hpp
            logging.cpp logging.hpp
            types.cpp ${CMAKE_SOURCE_DIR}/include/types.hpp
            helpers.hpp helpers.cpp
            utils/clock.cpp utils/clock.hpp
            ros/ros-client.cpp ros/ros-client.hpp
            ros/opt_msgs/ArcoreCameraFeatures.h
            ros/opt_msgs/KeyPoint.h
            ros/opt_msgs/OptarNtpMessage.h)

set(CMAKE_OSX_ARCHITECTURES "$(ARCHS_STANDARD_64_BIT)")
set(CMAKE_XCODE_EFFECTIVE_PLATFORMS "-iphoneos;-iphonesimulator")

if (IOS)
  set(SOURCES ${SOURCES}
     ros/objc/ros-ios.mm ros/objc/ros-ios.hpp)
endif()


add_library(${LIBRARY_NAME} STATIC ${SOURCES})
target_compile_features(${LIBRARY_NAME} PUBLIC cxx_std_17)

add_compile_definitions(
  $<$<CONFIG:DEBUG>:DEBUG>
)

#set(CMAKE_BUILD_TYPE Release)

target_include_directories(${LIBRARY_NAME}
    PRIVATE
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    ${CMAKE_SOURCE_DIR}/src/ros
    ${CMAKE_SOURCE_DIR}/src/utils)
target_include_directories(${LIBRARY_NAME}
    SYSTEM PRIVATE ${CMAKE_SOURCE_DIR}/thirdparty)

# add dependencies
# spdlog
#find_library(SPDLOG_LIBRARY spdlog REQUIRED NO_CMAKE_FIND_ROOT_PATH PATHS ${CMAKE_SOURCE_DIR}/thirdparty/spdlog/include)
set(SPDLOG_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/thirdparty/spdlog/include)
set(HAVE_SPDLOG 1)
target_include_directories(${LIBRARY_NAME} SYSTEM PRIVATE ${SPDLOG_INCLUDE_DIR})
#target_link_libraries(${LIBRARY_NAME} ${SPDLOG_LIBRARY})

# opencv
set(CMAKE_FRAMEWORK_PATH ${CMAKE_SOURCE_DIR}/thirdparty)
find_library(OPENCV_LIBRARY NAMES opencv2 REQUIRED NO_CMAKE_FIND_ROOT_PATH)
target_link_libraries(${LIBRARY_NAME} ${OPENCV_LIBRARY})

# ROS
find_library(ROS_LIBRARY NAMES ros REQUIRED NO_CMAKE_FIND_ROOT_PATH)
target_link_libraries(${LIBRARY_NAME} ${ROS_LIBRARY})

find_library(BOOST_LIBRARY NAMES boost REQUIRED NO_CMAKE_FIND_ROOT_PATH)
target_link_libraries(${LIBRARY_NAME} ${BOOST_LIBRARY})

find_library(ROS_TF_LIBRARY NAMES tf REQUIRED NO_CMAKE_FIND_ROOT_PATH)
target_link_libraries(${LIBRARY_NAME} ${ROS_TF_LIBRARY})

find_library(ROS_TF2_LIBRARY NAMES tf2 REQUIRED NO_CMAKE_FIND_ROOT_PATH)
target_link_libraries(${LIBRARY_NAME} ${ROS_TF2_LIBRARY})

find_library(ROS_TF2ROS_LIBRARY NAMES tf2_ros REQUIRED NO_CMAKE_FIND_ROOT_PATH)
target_link_libraries(${LIBRARY_NAME} ${ROS_TF2ROS_LIBRARY})

find_library(ROS_SENSOR_MSGS_LIBRARY NAMES sensor_msgs REQUIRED NO_CMAKE_FIND_ROOT_PATH)
target_link_libraries(${LIBRARY_NAME} ${ROS_SENSOR_MSGS_LIBRARY})

find_library(ROS_NAV_MSGS_LIBRARY NAMES nav_msgs REQUIRED NO_CMAKE_FIND_ROOT_PATH)
target_link_libraries(${LIBRARY_NAME} ${ROS_NAV_MSGS_LIBRARY})

find_library(ROS_STD_MSGS_LIBRARY NAMES std_msgs REQUIRED NO_CMAKE_FIND_ROOT_PATH)
target_link_libraries(${LIBRARY_NAME} ${ROS_STD_MSGS_LIBRARY})

find_library(ROS_GEO_MSGS_LIBRARY NAMES geometry_msgs REQUIRED NO_CMAKE_FIND_ROOT_PATH)
target_link_libraries(${LIBRARY_NAME} ${ROS_GEO_MSGS_LIBRARY})

find_library(ROS_ROSGRAPH_MSGS_LIBRARY NAMES rosgraph_msgs REQUIRED NO_CMAKE_FIND_ROOT_PATH)
target_link_libraries(${LIBRARY_NAME} ${ROS_ROSGRAPH_MSGS_LIBRARY})

find_library(ROS_TF2_MSGS_LIBRARY NAMES tf2_msgs REQUIRED NO_CMAKE_FIND_ROOT_PATH)
target_link_libraries(${LIBRARY_NAME} ${ROS_TF2_MSGS_LIBRARY})

# config
configure_file(${CMAKE_SOURCE_DIR}/optar-config.hpp.in ${CMAKE_SOURCE_DIR}/include/config.hpp)
