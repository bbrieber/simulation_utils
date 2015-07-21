cmake_minimum_required(VERSION 2.8.3)
project(iai_gazebo_utils)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  rospy
  iai_gazebo_msgs
  designator_integration_cpp
  designator_integration_msgs
)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
#  pkg_check_modules(OGRE OGRE)
#  pkg_check_modules(OGRE-Terrain OGRE-Terrain)
endif()

find_package(Protobuf REQUIRED)

include_directories(SYSTEM ${GAZEBO_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR}/gz_msgs)
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_subdirectory(gz_msgs)

link_directories(${GAZEBO_LIBRARY_DIRS} ${CMAKE_CURRENT_BINARY_DIR}/gz_msgs)
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES mocap_gazebo
#  CATKIN_DEPENDS gazebo_msgs gazebo_ros roscpp std_msgs tf2
#  DEPENDS system_lib
)



add_library(iai_object_scanner
   src/ObjectScanner.cpp
)

add_library(iai_object_transmitter
   src/ObjectTransmitter.cpp
)

add_library(iai_battery_consumer
   src/BatteryConsumer.cpp
)

target_link_libraries(iai_object_scanner 
iai_gazebo_internal_scan_msgs
    ${GAZEBO_LIBRARIES}
    ${PROTOBUF_LIBRARIES}
    ${catkin_LIBRARIES}
    )
    
target_link_libraries(iai_object_transmitter 
    iai_gazebo_internal_scan_msgs
    ${GAZEBO_LIBRARIES}
    ${PROTOBUF_LIBRARIES}
    ${catkin_LIBRARIES}
    )
    
target_link_libraries(iai_battery_consumer 
    ${GAZEBO_LIBRARIES})
    
    
add_dependencies(iai_battery_consumer ${catkin_EXPORTED_TARGETS})
    
add_dependencies(iai_object_transmitter iai_gazebo_internal_scan_msgs ${catkin_EXPORTED_TARGETS})
    
add_dependencies(iai_object_scanner iai_gazebo_internal_scan_msgs ${catkin_EXPORTED_TARGETS})