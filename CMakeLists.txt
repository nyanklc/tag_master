cmake_minimum_required(VERSION 3.0.2)
project(tag_master)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  cv_bridge
  message_generation
  cmake_modules
)

find_package(cmake_modules REQUIRED)
find_package(OpenCV 3.2.0 REQUIRED)
find_package(TinyXML REQUIRED)

list(APPEND CMAKE_PREFIX_PATH "${PROJECT_SOURCE_DIR}/apriltag")
add_subdirectory(./apriltag)
set(apriltag_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/apriltag)
link_directories(${PROJECT_SOURCE_DIR}/apriltag)

add_message_files(
  FILES
  TagPose.msg
)
add_service_files(
  FILES
  AddDetector.srv
  AddTagDescription.srv
  DebugCall.srv
  EnableDetector.srv
)
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${apriltag_INCLUDE_DIRS}
  LIBRARIES
    apriltag
    tag_master
    detector_base
    apriltag_detector
    utils
    ${apriltag_LIBRARIES}
  CATKIN_DEPENDS
    sensor_msgs
    geometry_msgs
    visualization_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    tf2_eigen
    cv_bridge
    cmake_modules
  DEPENDS
    OpenCV
    TinyXML
    apriltag
)

include_directories(
 ${PROJECT_SOURCE_DIR}/include
 ${PROJECT_SOURCE_DIR}/apriltag
 ${catkin_INCLUDE_DIRS}
 ${OpenCV_INCLUDE_DIRS}
 ${TinyXML_INCLUDE_DIRS}
 ${apriltag_INCLUDE_DIRS}
)

add_library(utils src/utils.cpp)
target_link_libraries(utils ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} apriltag)

add_library(detector_base src/detector_base.cpp)
target_link_libraries(detector_base ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} utils)

add_library(apriltag_detector src/apriltag_detector.cpp)
target_link_libraries(apriltag_detector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} utils detector_base apriltag)

add_library(tag_master src/tag_master.cpp)
target_link_libraries(tag_master ${catkin_LIBRARIES} detector_base apriltag_detector utils)

add_executable(tag_master_node src/tag_master_node.cpp)
target_link_libraries(tag_master_node ${catkin_LIBRARIES} tag_master detector_base apriltag_detector utils)