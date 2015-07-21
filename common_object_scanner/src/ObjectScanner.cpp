#include "common_object_scanner/ObjectScanner.hpp"
#include "../build/gz_msgs/iai_scan_msgs.pb.h"
#include <designator_integration_msgs/Designator.h>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/physics/PhysicsIface.hh>

#include "sdf/sdf.hh"

#include <boost/algorithm/string.hpp>

void gazebo::IAI_ObjectScanner::Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  
  ///ROS INITIALISATION
      if (!ros::isInitialized())
      {
	ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
	return;
      }
      
      
      
      namespace_ ="";
      topic_ = "obj_scan";
      frame_id_ = "map";
      if (_sdf->HasElement("robotNamespace")) 
	namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
      if (_sdf->HasElement("frame_id")) 
	frame_id_ = _sdf->GetElement("frame_id")->Get<std::string>();
      if (_sdf->HasElement("topic")) 
	topic_ = _sdf->GetElement("topic")->Get<std::string>();
      
      n = ros::NodeHandle(namespace_);
      //detection_pub = n.advertise<sim_utils_msgs::ObjectDetection>(topic_, 1);
      detection_pub = n.advertise<designator_integration_msgs::Designator>(topic_, 1);
      
    
      
      
  ///GAZEBO INIT
      
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&IAI_ObjectScanner::OnUpdate, this, _1));
      this->sensor = _sensor;
      this->name = _sensor->GetScopedName();
      
      update_rate = 1;
      scanner_id = 1235;
      range = 13.3;
      target_type = "all";
      name = "robot";
      
      if (_sdf->GetElement("update_rate"))
	this->update_rate = _sdf->GetElement("update_rate")->Get<double>();
      if (_sdf->GetElement("scanner_id"))
	this->scanner_id = _sdf->GetElement("scanner_id")->Get<int>();
      if (_sdf->GetElement("range"))
	this->range = _sdf->GetElement("range")->Get<double>();
      if (_sdf->GetElement("target_type"))
	this->target_type = _sdf->GetElement("target_type")->Get<std::string>();

      


      last_time  = 0;      
      node = transport::NodePtr(new transport::Node());
      this->node->Init(IAI_SCAN_NODE_NAME);
  
//std::string parentName = sensor->GetParentName();
//physics::EntityPtr parent = world->GetEntity(parentName);
  
  scanRequestPublisher = this->node->Advertise<iai_gazebo_msgs::gz_msgs::iai_scan_ping>(IAI_SCAN_PING_TOPIC); 
  scanResponseSubscriber = this->node->Subscribe(IAI_SCAN_PONG_TOPIC, &IAI_ObjectScanner::onPong, this);
  
  //this->updateConnection2 = this->sensor->ConnectUpdated(boost::bind(&IAI_ObjectScanner::OnUp2, this));
  sensor->SetActive(true);
  
  world = gazebo::physics::get_world( sensor->GetWorldName() );
  std::string parentName = sensor->GetParentName();
  parent = world->GetEntity(parentName);
}


void gazebo::IAI_ObjectScanner::OnUp2(){
  
	//std::cout << "BLAAAAA " << std::endl;
}
void gazebo::IAI_ObjectScanner::OnUpdate(const gazebo::common::UpdateInfo& info)
{
      //std::cout << "update" << info.simTime.Double() << std::endl;
      current_time = info.simTime.Double();
      double time_diff = current_time-last_time; 
      if(update_rate< time_diff){
	//IAI_Scan_Ping ping;
	//sensor->Update(true);
	//sensor->GetParentId();
	pose = sensor->GetPose() + parent->GetWorldPose();
	iai_gazebo_msgs::gz_msgs::iai_scan_ping ping;
	ping.set_x(pose.pos.x);
	ping.set_y(pose.pos.y);
	ping.set_z(pose.pos.z);
	ping.set_id(scanner_id);
	ping.set_stamp_id(1);
	ping.set_range(range);
	ping.set_type(target_type);
	scanRequestPublisher->Publish(ping);
	last_time = current_time;
      }
}

void gazebo::IAI_ObjectScanner::onPong(gazebo::IAI_Scan_Pong& msg)
{
    if(msg->id() != scanner_id)
      return;
    designator_integration::Designator *detection = new designator_integration::Designator();
    detection->setType(designator_integration::Designator::OBJECT);
    detection->setValue("OBJECTID",msg->id());
    //designator_integration::KeyValuePair* ckvpTest1 = detection->addChild(
    
    for (int i = 0; i < msg->attributes_size(); i++){
      const iai_gazebo_msgs::gz_msgs::iai_attribute& attr= msg->attributes(i);
      
      detection->setValue(boost::to_upper_copy<std::string>(attr.key()),attr.value());
    }
    
    if(msg->has_world_pose()){
      geometry_msgs::PoseStamped p_stamped;
      p_stamped.header.stamp = ros::Time::now();
      p_stamped.header.frame_id = frame_id_;
      
      p_stamped.pose.position.x = msg->world_pose().lx();
      p_stamped.pose.position.y = msg->world_pose().ly();
      p_stamped.pose.position.z = msg->world_pose().lz();
      p_stamped.pose.orientation.x = msg->world_pose().rx();
      p_stamped.pose.orientation.y = msg->world_pose().ry();
      p_stamped.pose.orientation.z = msg->world_pose().rz();
      p_stamped.pose.orientation.w = msg->world_pose().rw();
      detection->setValue("POSE",p_stamped);
      
    }else{
      geometry_msgs::PoseStamped p_stamped;
      p_stamped.header.stamp = ros::Time::now();
      p_stamped.header.frame_id = frame_id_;
      
      p_stamped.pose.position.x = pose.pos.x;
      p_stamped.pose.position.y = pose.pos.y;
      p_stamped.pose.position.z = pose.pos.z;
      p_stamped.pose.orientation.x = pose.rot.x;
      p_stamped.pose.orientation.y = pose.rot.y;
      p_stamped.pose.orientation.z = pose.rot.z;
      p_stamped.pose.orientation.w = pose.rot.w;
      detection->setValue("SCAN_POSE",p_stamped);
      detection->setValue("SIGNAL_STRENGTH",msg->strength());          
    }
    detection_pub.publish(detection->serializeToMessage());
    detection->printDesignator();
    //std::cout <<"fppppp"<<std::endl;
    //iai_gazebo_msgs::ObjectDetection detection;
    /**
    detection.object_type = msg->type();
    detection.strength = msg->strength();
    detection.robot_name =name;
    
    detection.pose = p_stamped;
    detection_pub.publish<iai_gazebo_msgs::ObjectDetection>(detection);
    **/
//	std::cout << "pong " << std::endl;
}
