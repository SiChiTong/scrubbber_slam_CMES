cmake_minimum_required(VERSION 2.8.3)
project(lads)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs	
  sensor_msgs
  message_generation
  common_msgs
  geometry_msgs
)

add_message_files(
    FILES
    ivsensorgps.msg
    ivsensorimu.msg
    ivwheelspeed.msg
 )

 generate_messages(
   DEPENDENCIES
   std_msgs
   common_msgs
   geometry_msgs
 )

catkin_package(
   CATKIN_DEPENDS roscpp rospy std_msgs
   message_runtime
   common_msgs
   geometry_msgs
)

include_directories(include 
   ${catkin_INCLUDE_DIRS}
)  
