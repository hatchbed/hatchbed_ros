cmake_minimum_required(VERSION 3.0.2)
project(hatchbed_localization_util)

find_package(catkin REQUIRED
  geometry_msgs
  nav_msgs
  nodelet
  param_util
  roscpp
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    nav_msgs
    nodelet
    param_util
    roscpp
    sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_nodelets
  src/nodelets/imu_to_twist.cpp
  src/nodelets/odom_to_twist.cpp
)
set_property(TARGET ${PROJECT_NAME}_nodelets PROPERTY CXX_STANDARD 17)
target_link_libraries(${PROJECT_NAME}_nodelets ${catkin_LIBRARIES})

install(
  TARGETS
    ${PROJECT_NAME}_nodelets
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(
  FILES
    nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})