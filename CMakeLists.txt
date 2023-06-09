cmake_minimum_required(VERSION 2.8.3)
project(xarm7_moveit_config)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_core
  moveit_ros_planning_interface
  tf
 xarm_msgs
xarm_api
)
catkin_package(
  CATKIN_DEPENDS
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(waypoint scripts/pickandplace_waypoint.cpp)
target_link_libraries(waypoint ${catkin_LIBRARIES})

add_executable(hapkit scripts/hapkit_ros.cpp)
target_link_libraries(hapkit ${catkin_LIBRARIES})

add_executable(hapkit1 scripts/hapkit_1.cpp)
target_link_libraries(hapkit1 ${catkin_LIBRARIES})

add_executable(pose scripts/publishpose.cpp)
target_link_libraries(pose ${catkin_LIBRARIES})

add_executable(read scripts/read.cc)
target_link_libraries(read ${catkin_LIBRARIES})


install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(TARGETS
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



