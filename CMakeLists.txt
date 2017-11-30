cmake_minimum_required(VERSION 2.8.3)
project(selidar_node)

FILE(GLOB SELIDAR_DRV_SRC 
  "./src/drv/*.cpp"
  "./src/utils/*.cpp"
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosconsole
  sensor_msgs
  message_generation
  rospy
  std_msgs
  std_srvs
)

include_directories(
  ./src/utils
  ./src/drv
  ${catkin_INCLUDE_DIRS}
)


generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package()
add_executable(selidarNode src/node.cpp ${SELIDAR_DRV_SRC})
target_link_libraries(selidarNode ${catkin_LIBRARIES})

install(TARGETS selidarNode
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch rviz sdk
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
