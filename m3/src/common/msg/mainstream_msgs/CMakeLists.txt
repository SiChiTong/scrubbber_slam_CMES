cmake_minimum_required(VERSION 2.8.3)
project(mainstream_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  common_msgs
  ivlocmsg
  ivcanbus
  geometry_msgs
)

add_message_files(
  FILES
  ivmsglocalization.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  common_msgs
  ivlocmsg
  ivcanbus
)

catkin_package(
 CATKIN_DEPENDS message_generation roscpp rospy std_msgs common_msgs
  ivlocmsg
  ivcanbus
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

