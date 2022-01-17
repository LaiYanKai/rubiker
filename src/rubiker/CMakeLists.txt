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

add_message_files(
  FILES
  MotorCmd.msg
  MotorAck.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(motor src/motor.cpp)
target_link_libraries(motor ${catkin_LIBRARIES} wiringPi -lrt -lpthread)
add_dependencies(motor ${{PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(master src/master.cpp)
target_link_libraries(master ${catkin_LIBRARIES} wiringPi -lrt)
add_dependencies(master ${{PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )
#if (${ARCHITECTURE} MATCHES "arm")
#    target_link_libraries(rubiker ${catkin_LIBRARIES} wiringPi -lrt)
#else()
#    target_link_libraries(rubiker ${catkin_LIBRARIES})
#endif()
