cmake_minimum_required(VERSION 2.8.3)
project(ivactuator)
set(CMAKE_CXX_FLAGS " ${CMAKE_CXX_FLAGS} -Wall")
find_package(
	catkin REQUIRED COMPONENTS 
	roscpp
	rospy 
	std_msgs 
	genmsg 
	monitor_msgs
  geometry_msgs
)

add_message_files(FILES
 ivactuator.msg 
 ivwheelspeed.msg 
 ivvehicle.msg
 )

generate_messages(
	DEPENDENCIES 
	std_msgs 
	monitor_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
  DEPENDS 
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}
)

