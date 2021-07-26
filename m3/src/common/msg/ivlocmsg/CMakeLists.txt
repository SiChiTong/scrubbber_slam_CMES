cmake_minimum_required(VERSION 2.8.3)
project(ivlocmsg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs	
  sensor_msgs
  message_generation
  monitor_msgs
)

add_message_files(
    FILES
    ivsensorgps.msg
    ivsensorodom.msg
    ndt_status.msg
    ivmsglocpos.msg
    ivmsgeuler.msg
    ivsensorimu.msg
    ivcalib.msg
    ivmsglocpos_with_fault.msg
 )

 generate_messages(
   DEPENDENCIES
   std_msgs
   monitor_msgs
 )

catkin_package(
   CATKIN_DEPENDS roscpp rospy std_msgs
   message_runtime
   monitor_msgs
)

include_directories(include 
   ${catkin_INCLUDE_DIRS}
)  
