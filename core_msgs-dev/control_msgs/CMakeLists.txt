cmake_minimum_required(VERSION 2.8.3)
project(control_msgs)

set(DEPEND_MSGS )

find_package(catkin REQUIRED
  COMPONENTS
    std_msgs
    message_generation
)

add_message_files(
  DIRECTORY
    msg
  FILES
   InCommand.msg
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
