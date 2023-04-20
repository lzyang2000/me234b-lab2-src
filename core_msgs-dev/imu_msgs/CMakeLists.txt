cmake_minimum_required(VERSION 2.8.3)
project(imu_msgs)

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
    imu_sync_info.msg
    imu_euler.msg
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
