#ifndef __IAI_OBJECT_SCANNER_HPP__
#define __IAI_OBJECT_SCANNER_HPP__

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/math/Pose.hh>

#include <gazebo/physics/World.hh>
#include <gazebo/physics/Entity.hh>


#include <ros/ros.h>
#include "sim_utils_msgs/ObjectDetection.h"

#include "ScanCommon.hpp"

#include "designators/Designator.h"
#include "designators/KeyValuePair.h"
#include "designator_integration_msgs/Designator.h"
#include "designator_integration_msgs/DesignatorCommunication.h"
namespace gazebo
{
  class IAI_ObjectScanner : public SensorPlugin{
    void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);
    void onPong(IAI_Scan_Pong &msg);
    void OnUpdate(const common::UpdateInfo & /*_info*/);
    void OnUp2();
  private:
    event::ConnectionPtr updateConnection2;
    transport::NodePtr node;
    transport::PublisherPtr scanRequestPublisher;
    transport::SubscriberPtr scanResponseSubscriber;
    event::ConnectionPtr updateConnection;
    
    
    sensors::SensorPtr sensor;
    
    double current_time;
    double last_time;
    double update_rate;
    
    int scanner_id;
    double range;
    std::string name;
    std::string target_type;
    std::string namespace_;
    std::string frame_id_;
    std::string topic_;
    
    math::Pose pose;
    physics::WorldPtr world;
    physics::EntityPtr parent;
    
    
    ros::NodeHandle n;
    ros::Publisher detection_pub;
    sim_utils_msgs::ObjectDetection detection;
  };
  GZ_REGISTER_SENSOR_PLUGIN(IAI_ObjectScanner)
}



#endif