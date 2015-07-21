#include "common_object_scanner/ObjectTransmitter.hpp"

#include <string.h>

void gazebo::IAI_ObjectTransmitter::Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{

  node = transport::NodePtr(new transport::Node());
  this->node->Init(IAI_SCAN_NODE_NAME);
      
  scanResponsePublisher = this->node->Advertise<iai_gazebo_msgs::gz_msgs::iai_scan_pong>(IAI_SCAN_PONG_TOPIC); 
  scanRequestSubscriber = this->node->Subscribe(IAI_SCAN_PING_TOPIC, &IAI_ObjectTransmitter::onPing, this);


  object_type = "victim";
  if (_sdf->GetElement("object_type"))
    this->object_type = _sdf->GetElement("object_type")->Get<std::string>();
  detectable = false;
  if (_sdf->GetElement("absolute_detection"))
    this->detectable = _sdf->GetElement("absolute_detection")->Get<bool>();
  
  sdf::ElementPtr elem = _sdf->GetElement("attribute");
  while (elem)
    {
      std::string key = elem->GetAttribute("key")->GetAsString();
      std::string value = elem->GetAttribute("value")->GetAsString();
      attributes[key] = value;
      std::cout << "key: " << key << " value:" << value <<std::endl;
      elem = elem->GetNextElement("attribute");
    }
  model = _parent;
  world = model->GetWorld();
//    gazPositionPub = this->gazNode->Advertise<sherpa_gazebo::msgs::sherpa_position>(SHERPA_GAZEBO_POSITION_TOPIC);  
}

typedef std::map<std::string,  std::string>::iterator it_attr_type;

#include "gazebo/msgs/msgs.hh"
//#include "pose.pb.h"
//#include "vector3d.pb.h"
//#include "quaternion.pb.h"

void gazebo::IAI_ObjectTransmitter::onPing(gazebo::IAI_Scan_Ping& msg)
{

      if(! validType(msg->type()))
	return;
      gazebo::math::Pose pose = model->GetWorldPose();
//      gazebo::math::Pose *sensorPose = new gazebo::math::Pose(msg->x(),msg->y(),msg->z(),0,0,0);
      double dist = pose.pos.Distance(msg->x(),msg->y(),msg->z());
//      std::cout << "Distance:"<< dist << std::endl ;
      if(dist< msg->range()){
	iai_gazebo_msgs::gz_msgs::iai_scan_pong pong;
	pong.set_strength((msg->range()-dist)/msg->range());
	pong.set_id(msg->id());
	pong.set_stamp_id(msg->stamp_id());
	pong.set_type(object_type);
	pong.set_type(object_type);
	
	for(it_attr_type iterator = attributes.begin(); iterator != attributes.end(); iterator++) {
	  iai_gazebo_msgs::gz_msgs::iai_attribute* attr = pong.add_attributes();
	  attr->set_key(iterator->first); 
	  attr->set_value(iterator->second);
	  
    	}
    	if(detectable){
	  //gazebo::msgs::Pose p;
	  //p.set_poistion();
	  /**
	  p.position().set_x(pose.pos.x);
	  p.position().set_y(pose.pos.y);
	  p.position().z =  pose.pos.z;
	  
	  p.set_orientation();
	  p.orientation().x =  pose.rot.x;
	  p.orientation().y =  pose.rot.y;
	  p.orientation().z =  pose.rot.z;
	  p.orientation().w =  pose.rot.w;
	  **/
	  iai_gazebo_msgs::gz_msgs::iai_pose p;
	  
	  p.set_lx(pose.pos.x);
	  p.set_ly(pose.pos.y);
	  p.set_lz(pose.pos.z);
	  
	  p.set_rx(pose.rot.x);
	  p.set_ry(pose.rot.y);
	  p.set_rz(pose.rot.z);
	  p.set_rw(pose.rot.w);
	  pong.set_allocated_world_pose(&p);
	}
	scanResponsePublisher->Publish(pong);
      }
      //gazebo::physics::BasePtr sensor = world->GetByName(msg->id());
      
//      gazebo::math::Pose = model->Get;
      model->GetWorld();
}

bool gazebo::IAI_ObjectTransmitter::validType(std::string types)
{
  if(types.compare("all") == 0 ){
//      std::cout <<"Scanning for all" << std::endl;
    return true;
  }
  std::string delm = ",";
  std::size_t pos = 0;
  std::string token;
  while ((pos = types.find(delm)) != std::string::npos) {
    token = types.substr(0, pos);
    if(token.compare(object_type) == 0 ){
//      std::cout <<"found match: " << token << std::endl;
      return true;
    }
//    std::cout <<"ignoring type: " << token << std::endl;
    types.erase(0, pos + delm.length());
  }
  if(types.compare(object_type) == 0 ){
//      std::cout <<"found match: " << types << std::endl;
      return true;
  }
//    std::cout <<"found no match: "<< std::endl;
  return false;
}
