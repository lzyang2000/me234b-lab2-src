cmake_minimum_required(VERSION 2.8.3)
project(iop_bridge)

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
    jaus_battery_status.msg
    jaus_gas_engine_status.msg
    jaus_diesel_engine_config.msg
    jaus_hydraulic_status.msg
    jaus_diesel_engine_status.msg
    jaus_stream_config.msg
    jaus_fuel_cell_status.msg
    jaus_visual_sensor_config.msg
    jaus_gas_engine_config.msg

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
