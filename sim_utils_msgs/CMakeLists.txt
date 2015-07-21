cmake_minimum_required(VERSION 2.8.3)
project(sim_utils_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  message_generation
  actionlib_msgs
)

add_message_files(
   FILES
   ObjectDetection.msg#just a placeholder for now
   IAI_Robot.msg#just a placeholder for now
)

add_service_files(
   FILES
   AddBatteryConsumer.srv
   RemoveBatteryConsumer.srv
)

add_action_files(
  DIRECTORY action
  FILES RechargeBattery.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs  
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime actionlib_msgs
)