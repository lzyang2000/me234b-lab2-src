cmake_minimum_required(VERSION 2.8.3)
project(gvrbot)

set(DEPEND_MSGS )

find_package(catkin REQUIRED
  COMPONENTS
    std_msgs
    geometry_msgs
    message_generation
)

add_message_files(
  DIRECTORY
    msg
  FILES
   GvrbotComms.msg
   GvrbotFlipperData.msg
   GvrbotFlipperEffort.msg
   GvrbotMobilityData.msg
   GvrbotOrientation.msg
   GvrbotPowerData.msg
   GvrbotState.msg
   GvrbotFlipperPosition.msg
   GvrbotFlipperValues.msg
   GvrbotMobilityValues.msg

)

generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
)
