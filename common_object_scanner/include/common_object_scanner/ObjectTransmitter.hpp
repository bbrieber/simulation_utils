#ifndef __IAI_OBJECT_TRANSMITTER_HPP__
#define __IAI_OBJECT_TRANSMITTER_HPP__

#include <gazebo/common/Plugin.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include "ScanCommon.hpp"

namespace gazebo
{
  class IAI_ObjectTransmitter : public ModelPlugin{
  public:
    void Load(physics::ModelPtr _sensor, sdf::ElementPtr _sdf);
    void onPing(IAI_Scan_Ping &msg);
  private:
    bool validType(std::string types);
    
    transport::NodePtr node;
    transport::PublisherPtr scanResponsePublisher;
    transport::SubscriberPtr scanRequestSubscriber;
	  
    
    physics::ModelPtr model;
    physics::WorldPtr world;
    
    std::map<std::string,std::string> attributes;
    bool detectable;
    std::string object_type;
      
    
  };
  GZ_REGISTER_MODEL_PLUGIN(IAI_ObjectTransmitter)
}

#endif