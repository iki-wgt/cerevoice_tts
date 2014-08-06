cmake_minimum_required(VERSION 2.8.3)
project(cerevoice_tts_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  message_generation
  std_msgs
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  Tts.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
)

catkin_package(
 CATKIN_DEPENDS actionlib_msgs message_runtime std_msgs
)
