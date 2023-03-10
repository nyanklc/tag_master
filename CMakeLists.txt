cmake_minimum_required(VERSION 3.0.2)
project(tag_master)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  cv_bridge
)

find_package(OpenCV 3.2.0 REQUIRED)
find_package(apriltag REQUIRED)

add_message_files(
  FILES
  TagPose.msg
  RobotPoseEstimate.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
    tag_master
    detector_base
    apriltag_detector
    utils
  CATKIN_DEPENDS 
    sensor_msgs
    geometry_msgs
    visualization_msgs
    tf2
    tf2_ros
    cv_bridge
  DEPENDS
    apriltag
    OpenCV
    TinyXML
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${OpenCV_INCLUDE_DIRS}
 ${apriltag_INCLUDE_DIRS}
 ${TinyXML_INCLUDE_DIRS}
)

add_library(utils src/utils.cpp)
add_dependencies(utils ${OpenCV_LIBS} ${catkin_LIBRARIES} apriltag)
target_link_libraries(utils ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} apriltag)

add_library(detector_base src/detector_base.cpp)
add_dependencies(detector_base ${OpenCV_LIBS} ${catkin_LIBRARIES})
target_link_libraries(detector_base ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_library(apriltag_detector src/apriltag_detector.cpp)
add_dependencies(apriltag_detector ${OpenCV_LIBS} ${apriltag_LIBRARIES} ${catkin_LIBRARIES} utils)
target_link_libraries(apriltag_detector ${catkin_LIBRARIES} ${apriltag_LIBRARIES} ${OpenCV_LIBRARIES} utils)

add_library(tag_master src/tag_master.cpp)
add_dependencies(tag_master ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} detector_base)
target_link_libraries(tag_master ${catkin_LIBRARIES} detector_base)

add_executable(tag_master_node src/tag_master_node.cpp)
add_dependencies(tag_master_node ${catkin_LIBRARIES} tag_master detector_base apriltag_detector utils)
target_link_libraries(tag_master_node tag_master ${catkin_LIBRARIES} detector_base apriltag_detector utils ${TinyXML_INCLUDE_DIRS})

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
