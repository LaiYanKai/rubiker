
cmake_minimum_required(VERSION 3.0.2)
project(rpi_cam_pub)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  cv_bridge
  geometry_msgs
  image_transport
  message_generation
  nav_msgs
  sensor_msgs
  std_msgs
  actionlib_msgs
  tf
)
catkin_package(
  CATKIN_DEPENDS 
    roscpp
    rospy
    cv_bridge
    geometry_msgs
    image_transport
    message_generation
    nav_msgs
    sensor_msgs
    std_msgs
    actionlib_msgs
    tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCv_INCLUDE_DIRS}
)

add_executable(rpi_cam_pub src/rpi_cam_pub.cpp)
  target_link_libraries(rpi_cam_pub ${catkin_LIBRARIES} ${OpenCV_INCLUDE_DIRS})
