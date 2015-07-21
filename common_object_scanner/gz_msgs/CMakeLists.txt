
find_package(Protobuf REQUIRED)

set(PROTOBUF_IMPORT_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
  endif()
endforeach()

set(scan_msgs 
  iai_scan_msgs.proto
  )
#set(scan_msgs iai_scan_ping.proto iai_scan_pong.proto)
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${scan_msgs})
add_library(iai_gazebo_internal_scan_msgs SHARED ${PROTO_SRCS})
target_link_libraries(iai_gazebo_internal_scan_msgs ${PROTOBUF_LIBRARY})