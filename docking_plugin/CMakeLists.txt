cmake_minimum_required(VERSION 2.8.3)
project(docking_plugin)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  rospy
)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
find_package(Boost REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

message("##############  OGRE START ###################")
message(${GAZEBO_INCLUDE_DIRS})
message("##############  OGRE END   ###################")

catkin_package(
  INCLUDE_DIRS include
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)



        

add_library(iai_box_docking
   src/iai_box_docking.cpp
)

target_link_libraries(iai_box_docking
   ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES}
 )
