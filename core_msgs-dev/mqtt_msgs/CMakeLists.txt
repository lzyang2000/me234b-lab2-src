cmake_minimum_required(VERSION 2.8.3)
project(mqtt_msgs)

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
    LincControlState.msg
    LincRisk.msg
    LincState.msg
    LincSystemState.msg
    MqttServerState.msg

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
