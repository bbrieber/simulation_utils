#ifndef __IAI_SCAN_COMMON_HPP__
#define __IAI_SCAN_COMMON_HPP__

#include "iai_scan_msgs.pb.h"
//#include "iai_scan_pong.pb.h"



#include <boost/shared_ptr.hpp>
#include <sdf/sdf.hh>
#include <boost/gil/gil_all.hpp>
#include <boost/gil/extension/io/png_dynamic_io.hpp>


#define IAI_SCAN_NODE_NAME "iai_scan"
#define IAI_SCAN_PING_TOPIC "~/scan_ping"
#define IAI_SCAN_PONG_TOPIC "~/scan_pong"

namespace gazebo
{
  
  typedef const boost::shared_ptr<const iai_gazebo_msgs::gz_msgs::iai_scan_ping> IAI_Scan_Ping;
  typedef const boost::shared_ptr<const iai_gazebo_msgs::gz_msgs::iai_scan_pong> IAI_Scan_Pong;
//  typedef const boost::shared_ptr<const sherpa_gazebo::msgs::sherpa_position> SherpaPositionPtr;
}

#endif