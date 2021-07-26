cmake_minimum_required(VERSION 2.8.3)
project(ivapp)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg
)
find_package(OpenCV REQUIRED)

add_message_files(
   FILES
   specialfunctionpoint.msg
   app2taskcentral.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs
  message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
