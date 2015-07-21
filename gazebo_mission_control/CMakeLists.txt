cmake_minimum_required(VERSION 2.8.3)
project(gazebo_mission_control)

find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  gazebo_ros
  roscpp
  std_msgs
  tf2
)


find_package (Qt4)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
  pkg_check_modules(OGRE OGRE)
  pkg_check_modules(OGRE-Terrain OGRE-Terrain)
endif()


find_package(Protobuf REQUIRED)

include (${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})

include_directories(SYSTEM ${GAZEBO_INCLUDE_DIRS} ${OGRE_INCLUDE_DIRS} ${OGRE-Terrain_INCLUDE_DIRS})
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)




link_directories(${GAZEBO_LIBRARY_DIRS} ${OGRE_LIBRARY_DIRS} ${OGRE-Terrain_LIBRARY_DIRS})
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES mocap_gazebo
#  CATKIN_DEPENDS gazebo_msgs gazebo_ros roscpp std_msgs tf2
#  DEPENDS system_lib
)


MESSAGE( STATUS "CMAKE_INCLUDE_DIR:         " ${CMAKE_INCLUDE_PATH} )

QT4_WRAP_CPP(headers_MOC include/gazebo_mission_control/OperatorOverlay.hpp)

add_library(iai_rescue_operator_overlay_gazebo
    SHARED ${headers_MOC}
   src/OperatorOverlay.cpp
)


target_link_libraries(iai_rescue_operator_overlay_gazebo ${GAZEBO_LIBRARIES}
    ${OGRE_LIBRARIES} 
    ${OGRE-Terrain_LIBRARIES}
    ${QT_LIBRARIES}
    ${PROTOBUF_LIBRARIES})
    
    
    
add_library(iai_rescue_operator_model_gazebo
   src/OperatorModel.cpp
)

target_link_libraries(iai_rescue_operator_model_gazebo ${GAZEBO_LIBRARIES}
    ${PROTOBUF_LIBRARIES})
    

add_library(iai_rescue_operator_vis_gazebo
   src/OperatorVisualization.cpp
)

target_link_libraries(iai_rescue_operator_vis_gazebo ${GAZEBO_LIBRARIES}
    ${PROTOBUF_LIBRARIES})
    