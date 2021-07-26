cmake_minimum_required(VERSION 2.8.3)
project(new_ivapp)

find_package(catkin REQUIRED COMPONENTS 
                 message_generation 
                 std_msgs 
                 )

add_message_files(
        DIRECTORY msg
        FILES
        specialfunctionpoint.msg
	app2taskcentral.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
)

catkin_package(
        CATKIN_DEPENDS
        message_runtime
        std_msgs
)
