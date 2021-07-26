cmake_minimum_required(VERSION 2.8.3)
project(suteng_irad)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

add_message_files(
        DIRECTORY msg
        FILES
        UdpPackets.msg
        LidarPackets.msg
)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(
        CATKIN_DEPENDS message_runtime std_msgs
)
