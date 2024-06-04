# Copyright 2016 The Cartographer Authors
# Copyright 2022 Wyca Robotics (for the ROS2 conversion)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)

project(cartographer_ros)

find_package(ament_cmake REQUIRED)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(builtin_interfaces REQUIRED)
find_package(cartographer REQUIRED)
find_package(cartographer_ros_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(absl REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${urdfdom_headers_INCLUDE_DIRS}
)

# Library
add_library(${PROJECT_NAME}
  src/assets_writer.cpp
  src/map_builder_bridge.cpp
  src/msg_conversion.cpp
  src/node_constants.cpp
  src/node.cpp
  src/node_options.cpp
  src/offline_node.cpp
  src/playable_bag.cpp
  src/ros_log_sink.cpp
  src/ros_map.cpp
  src/ros_map_writing_points_processor.cpp
  src/sensor_bridge.cpp
  src/submap.cpp
  src/tf_bridge.cpp
  src/time_conversion.cpp
  src/trajectory_options.cpp
  src/urdf_reader.cpp
  src/metrics/family_factory.cpp
  src/metrics/internal/family.cpp
  src/metrics/internal/histogram.cpp
  )

set(dependencies
  builtin_interfaces
  cartographer
  cartographer_ros_msgs
  geometry_msgs
  nav_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_eigen
  tf2_msgs
  tf2_ros
  visualization_msgs
  rosbag2_cpp
  rosbag2_storage
  urdf
  urdfdom
)

if(WIN32)
  set(Boost_USE_STATIC_LIBS FALSE)
endif()
# For yet unknown reason, if Boost is find_packaged() after find_package(cartographer),
# some Boost libraries including Thread are set to static, despite Boost_USE_STATIC_LIBS,
# which causes linking problems on windows due to shared/static Thread clashing.
# PCL also finds Boost. Work around by moving before find_package(cartographer).
find_package(Boost REQUIRED COMPONENTS system iostreams)
find_package(PCL REQUIRED COMPONENTS common)

find_package(cartographer REQUIRED)
include("${CARTOGRAPHER_CMAKE_DIR}/functions.cmake")
option(BUILD_GRPC "build features that require Cartographer gRPC support" false)
google_initialize_cartographer_project()
google_enable_testing()
set(CARTOGRAPHER_GMOCK_LIBRARIES ${GMOCK_LIBRARIES})

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

include(FindPkgConfig)

find_package(absl REQUIRED)
find_package(LuaGoogle REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(urdfdom_headers REQUIRED)
if(DEFINED urdfdom_headers_VERSION)
  if(${urdfdom_headers_VERSION} GREATER 0.4.1)
    add_definitions(-DURDFDOM_HEADERS_HAS_SHARED_PTR_DEFS)
  endif()
endif()

include_directories(
  ${urdfdom_headers_INCLUDE_DIRS}
)

# Override Catkin's GTest configuration to use GMock.
set(GTEST_FOUND TRUE)
set(GTEST_INCLUDE_DIRS ${GMOCK_INCLUDE_DIRS})
set(GTEST_LIBRARIES ${CARTOGRAPHER_GMOCK_LIBRARIES})

catkin_package(
  CATKIN_DEPENDS
    ${PACKAGE_DEPENDENCIES}
  DEPENDS
    # TODO(damonkohler): This should be here but causes Catkin to abort because
    # protobuf specifies a library '-lpthread' instead of just 'pthread'.
    # CARTOGRAPHER
    PCL
    EIGEN3
    Boost
    urdfdom_headers
  INCLUDE_DIRS "."
  LIBRARIES ${PROJECT_NAME}
)

file(GLOB_RECURSE ALL_SRCS "cartographer_ros/*.cc" "cartographer_ros/*.h")
file(GLOB_RECURSE ALL_TESTS "cartographer_ros/*_test.cc")
file(GLOB_RECURSE ALL_EXECUTABLES "cartographer_ros/*_main.cc")
file(GLOB_RECURSE ALL_GRPC_FILES "cartographer_ros/cartographer_grpc/*")
list(REMOVE_ITEM ALL_SRCS ${ALL_TESTS})
list(REMOVE_ITEM ALL_SRCS ${ALL_EXECUTABLES})
if (NOT ${BUILD_GRPC})
  list(REMOVE_ITEM ALL_SRCS ${ALL_GRPC_FILES})
  list(REMOVE_ITEM ALL_TESTS ${ALL_GRPC_FILES})
  list(REMOVE_ITEM ALL_EXECUTABLES ${ALL_GRPC_FILES})
endif()
add_library(${PROJECT_NAME} STATIC ${ALL_SRCS})
add_subdirectory("cartographer_ros")

target_link_libraries(${PROJECT_NAME} PUBLIC cartographer)

# Lua
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${LUA_INCLUDE_DIR})

# PCL
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${PCL_LIBRARIES})
set(BLACKLISTED_PCL_DEFINITIONS " -march=native -msse4.2 -mfpmath=sse ")
foreach(DEFINITION ${PCL_DEFINITIONS})
  list (FIND BLACKLISTED_PCL_DEFINITIONS "${DEFINITION}" DEFINITIONS_INDEX)
  if (${DEFINITIONS_INDEX} GREATER -1)
    continue()
  endif()
  set(TARGET_COMPILE_FLAGS "${TARGET_COMPILE_FLAGS} ${DEFINITION}")
endforeach()

# Eigen
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  "${EIGEN3_INCLUDE_DIR}")

# Boost
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  "${Boost_INCLUDE_DIRS}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${Boost_LIBRARIES})

# Catkin
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

# Add the binary directory first, so that port.h is included after it has
# been generated.
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

set(TARGET_COMPILE_FLAGS "${TARGET_COMPILE_FLAGS} ${GOOG_CXX_FLAGS}")
set_target_properties(${PROJECT_NAME} PROPERTIES
  COMPILE_FLAGS ${TARGET_COMPILE_FLAGS})

if (CATKIN_ENABLE_TESTING)
  foreach(TEST_SOURCE_FILENAME ${ALL_TESTS})
    get_filename_component(TEST_NAME ${TEST_SOURCE_FILENAME} NAME_WE)
    catkin_add_gtest(${TEST_NAME} ${TEST_SOURCE_FILENAME})
    # catkin_add_gtest uses a plain (i.e. no PUBLIC/PRIVATE/INTERFACE) call to
    # target_link_libraries. That forces us to do the same.
    target_link_libraries(${TEST_NAME} ${GMOCK_LIBRARIES} ${GTEST_MAIN_LIBRARIES})
    target_include_directories(${TEST_NAME} SYSTEM PUBLIC ${LUA_INCLUDE_DIR})
    target_link_libraries(${TEST_NAME} ${LUA_LIBRARIES})
    target_include_directories(${TEST_NAME} SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
    target_link_libraries(${TEST_NAME} ${catkin_LIBRARIES})
    add_dependencies(${TEST_NAME} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(${TEST_NAME} cartographer)
    target_link_libraries(${TEST_NAME} ${PROJECT_NAME})
    set_target_properties(${TEST_NAME} PROPERTIES COMPILE_FLAGS ${TARGET_COMPILE_FLAGS})
    # Needed for dynamically linked GTest on Windows.
    if (WIN32)
      target_compile_definitions(${TEST_NAME} PUBLIC -DGTEST_LINKED_AS_SHARED_LIBRARY)
    endif()
  endforeach()
endif()

install(DIRECTORY launch urdf configuration_files
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/tf_remove_frames.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Install source headers.
file(GLOB_RECURSE HDRS "cartographer_ros/*.h")
foreach(HDR ${HDRS})
  file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${HDR})
  get_filename_component(INSTALL_DIR ${REL_FIL} DIRECTORY)
  install(
    FILES
      ${HDR}
    DESTINATION
      include/${INSTALL_DIR}
  )
target_link_libraries(${PROJECT_NAME} cartographer ${PCL_LIBRARIES})

# Executables
add_executable(cartographer_node src/node_main.cpp)
target_link_libraries(cartographer_node ${PROJECT_NAME})
ament_target_dependencies(cartographer_node ${dependencies})

add_executable(cartographer_occupancy_grid_node src/occupancy_grid_node_main.cpp)
target_link_libraries(cartographer_occupancy_grid_node ${PROJECT_NAME})
ament_target_dependencies(cartographer_occupancy_grid_node ${dependencies})

add_executable(cartographer_offline_node src/offline_node_main.cpp)
target_link_libraries(cartographer_offline_node ${PROJECT_NAME})
ament_target_dependencies(cartographer_offline_node ${dependencies})

add_executable(cartographer_assets_writer src/assets_writer_main.cpp)
target_link_libraries(cartographer_assets_writer ${PROJECT_NAME})
ament_target_dependencies(cartographer_assets_writer ${dependencies})

add_executable(cartographer_pbstream_map_publisher src/pbstream_map_publisher_main.cpp)
target_link_libraries(cartographer_pbstream_map_publisher ${PROJECT_NAME})
ament_target_dependencies(cartographer_pbstream_map_publisher ${dependencies})

add_executable(cartographer_pbstream_to_ros_map src/pbstream_to_ros_map_main.cpp)
target_link_libraries(cartographer_pbstream_to_ros_map ${PROJECT_NAME})
ament_target_dependencies(cartographer_pbstream_to_ros_map ${dependencies})

add_executable(cartographer_rosbag_validate src/rosbag_validate_main.cpp)
target_link_libraries(cartographer_rosbag_validate ${PROJECT_NAME})
ament_target_dependencies(cartographer_rosbag_validate ${dependencies})

if($ENV{ROS_DISTRO} MATCHES "humble" OR $ENV{ROS_DISTRO} MATCHES "iron")
  target_compile_definitions(${PROJECT_NAME} PRIVATE PRE_JAZZY_SERIALIZED_BAG_MSG_FIELD_NAME)
  target_compile_definitions(cartographer_rosbag_validate PRIVATE PRE_JAZZY_SERIALIZED_BAG_MSG_FIELD_NAME)
endif()

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  cartographer_node
  cartographer_occupancy_grid_node
  cartographer_offline_node
  cartographer_assets_writer
  cartographer_pbstream_map_publisher
  cartographer_pbstream_to_ros_map
  cartographer_rosbag_validate
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY configuration_files urdf launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()

# Non converted bin:
#google_binary(cartographer_dev_pbstream_trajectories_to_rosbag
#  SRCS
#  dev/pbstream_trajectories_to_rosbag_main.cc
#)

#google_binary(cartographer_dev_rosbag_publisher
#  SRCS
#  dev/rosbag_publisher_main.cc
#)


#google_binary(cartographer_dev_trajectory_comparison
#  SRCS
#  dev/trajectory_comparison_main.cc
#)


## TODO(cschuet): Add support for shared library case.
#if (${BUILD_GRPC})
#  google_binary(cartographer_grpc_node
#    SRCS
#      cartographer_grpc/node_grpc_main.cc
#  )


#  google_binary(cartographer_grpc_offline_node
#    SRCS
#      cartographer_grpc/offline_node_grpc_main.cc
#  )

#  install(PROGRAMS
#    ../scripts/cartographer_grpc_server.sh
#    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#  )
#endif()
