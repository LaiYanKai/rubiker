cmake_minimum_required(VERSION 3.0.2)
project(rubiker)

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

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
add_executable(pid src/pid.cpp)
target_link_libraries(pid ${catkin_LIBRARIES} wiringPi -lrt -lpthread)

#add_executable(pid2 src/pid2.cpp)
#target_link_libraries(pid2 ${catkin_LIBRARIES} wiringPi -lrt)
add_executable(reset src/reset.cpp)
target_link_libraries(reset ${catkin_LIBRARIES} wiringPi -lrt)

add_executable(tacho src/tacho.cpp)
target_link_libraries(tacho ${catkin_LIBRARIES} wiringPi -lrt)

add_executable(master src/master.cpp)
target_link_libraries(master ${catkin_LIBRARIES} wiringPi -lrt)

#EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )
#if (${ARCHITECTURE} MATCHES "arm")
#    target_link_libraries(rubiker ${catkin_LIBRARIES} wiringPi -lrt)
#else()
#    target_link_libraries(rubiker ${catkin_LIBRARIES})
#endif()
