/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2015  Benjamin Brieber <email>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "docking_plugin/iai_box_docking.h"
#include <gazebo_plugins/gazebo_ros_utils.h>


void gazebo::IAI_BoxDocking::Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    this->model = _parent;
    //updateCollisionMask(false);
//    this->model->Update();
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }
    if (_sdf->HasElement("robotNamespace")){
      robot_namespace_ =_sdf->GetElement("robotNamespace")->GetValue()->GetAsString();
    }else{
      ROS_WARN("IAI_BoxDocking missing <robotNamespace>, "
          "defaults to \"%s\"", robot_namespace_.c_str());
      robot_namespace_ = "/";
    }
    
    /**
     * FIXME for general solution
    if (_sdf->HasElement("recharge_points")){
      robot_namespace_ =_sdf->GetElement("robotNamespace")->GetValue()->GetAsString();
    }else{
      robot_namespace_ = "/";
    }
    */

//    this->robot_namespace_ =  GetRobotNamespace(_parent, _sdf, "Box");
    n = new ros::NodeHandle(this->robot_namespace_);// + "/Attacher");
    genralDockRequestService = n->advertiseService("general_dock_request", &gazebo::IAI_BoxDocking::onGeneralDockRequest, this);
    waspDockRequestService = n->advertiseService("dock_wasp_request", &gazebo::IAI_BoxDocking::onDockQuadRequest, this);
    attachRequestService = n->advertiseService("attach_box_request", &gazebo::IAI_BoxDocking::onBoxDockRequest, this);
    //dockRequestService = n->advertiseService("dock_box_request", &gazebo::IAI_BoxDocking::onBoxDockRequest, this);

    
    boxLink = this->model->GetLink("base_footprint");
    
    ROS_INFO("PLUGIN INITIALIZED");
    updateConnection = 
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&gazebo::IAI_BoxDocking::onUpdate, this));
      
    world = model->GetWorld();  
    //FIXME read from _sdf
    updateRate = 0.05;
    tf_broadcaster.reset(new tf::TransformBroadcaster());
    
    fixed_frame = "world";
    tf_prefix = "box";
    
    base_frame = tf::resolve(tf_prefix,"base_footprint");
    
    
    dockingPoints =
    {
        {QUAD_FRONT_LEFT, "quadrotor_dock_left_front_link"},
        {QUAD_FRONT_RIGHT, "quadrotor_dock_right_front_link"},
        {QUAD_BACK_LEFT, "quadrotor_dock_left_back_link"},
        {QUAD_BACK_RIGHT, "quadrotor_dock_right_back_link"},
	
        {BOX_DOCK_TOP, "bottom_dock_link"},
        {BOX_DOCK_BOTTOM, "top_dock_link"},
	
        {GRIPPER_DOCK_FRONT, "gripper_front_link"},
        {GRIPPER_DOCK_TOP, "gripper_top_link"}	
    };
    
}

void gazebo::IAI_BoxDocking::updateCollisionMask(bool collide){
  
    physics::Link_V links = this->model->GetLinks();
    for (std::vector<physics::LinkPtr>::iterator lk = links.begin(); lk != links.end(); ++lk){
      physics::Collision_V cols = (*lk)->GetCollisions();
  //    physics::CollisionPtr p;
      for (std::vector<physics::CollisionPtr>::iterator cl = cols.begin(); cl != cols.end(); ++cl){
	physics::CollisionPtr p = (*cl);
	if(!collide){
	  p->SetCollideBits(BOX_COLLIDE_BIT);
	  p->SetCollision(false);
	  ROS_WARN("REMOVING ALL BITS");
	    
	}
	else
	  p->SetCollideBits(DEFAULT_COLLIDE_BIT);
	ROS_WARN("SETTING DEFAULT BIT");
	
	//p->Set
      }
    }
}

gazebo::math::Pose gazebo::IAI_BoxDocking::tfToGzPose(tf::StampedTransform transform)
{
  math::Quaternion rotation = math::Quaternion( transform.getRotation().w(), transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z());
  math::Vector3 origin = math::Vector3(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z()); //T_q^w  
  math::Pose pose = math::Pose(origin,rotation);
  return pose;
}

bool gazebo::IAI_BoxDocking::createFixedLink(std::string name, gazebo::physics::LinkPtr link, bool setAsParent=false)
{
  //physics::LinkPtr boxLink = targetModel->GetLink("base_footprint");
  
  //math::Pose tPose = targetModel->GetWorldPose();
  math::Pose offset;
//  math::Vector3 off_loc = math::Vector3(transfom.getOrigin().x(), transfom.getOrigin().y(), transfom.getOrigin().z());
  //math::Pose offset = tPose-pose;
  //targetModel->AttachStaticModel(sourceModel,offset);
  
  gazebo::physics::LinkPtr parent;
  gazebo::physics::LinkPtr child;
  if(setAsParent){
    parent = link;
    child = boxLink;
    //ROS_INFO("Disabling Model gravity");
    //model->SetGravityMode(false);
    gazebo::physics::InertialPtr in =  child->GetInertial();
    in->SetMass(0.00000000000001);//FIXME find out what is going on here RESET value
    child->SetInertial(in);
    child->UpdateMass();
  }else{
    
    
    parent = boxLink;
    child = link;
  }
  std::map<std::string, physics::JointPtr>::iterator got = connections.find(name);

  if ( got == connections.end() ){
    ROS_INFO("Creating new JOINT");
    physics::JointPtr fixedJoint = world->GetPhysicsEngine()->CreateJoint("revolute", this->model);
    fixedJoint->Load(parent,
		child, offset);
    fixedJoint->Init();
    fixedJoint->SetHighStop(0, 0);
    fixedJoint->SetLowStop(0, 0);
    connections[name] = fixedJoint;
  }
  
  gazebo::physics::JointPtr fixedJoint2 = connections[name];
  //fixedJoint->Attach();
  fixedJoint2->Attach(parent,child);  
  fixedJoint2->Update();
  return true;
}


bool gazebo::IAI_BoxDocking::destroyFixedLink(std::string name)
{
  std::map<std::string, physics::JointPtr>::iterator got = connections.find(name);

  if ( got != connections.end() ){
    ROS_INFO("Detaching Joint");
    gazebo::physics::JointPtr fixedJoint = connections[name];
    fixedJoint->Detach();
    fixedJoint->Update();
    
    //this->model->SetGravityMode(true);//FIXME
    //this->model->GetLink("base_footprint")->UpdateMass();
    //this->model->SetStatic();
  }else{
    ROS_WARN("No Such joint");
  }
  
  
  return true;
  //connections.erase(name);
}



std::string gazebo::IAI_BoxDocking::resolveDockName(gazebo::DockingPoint dp)
{
  return tf::resolve(tf_prefix,dockingPoints[dp]);;
}


//tf::StampedTransform gazebo::IAI_BoxDocking::getBoxOffset(gazebo::DockingPoint dp)
//{
// 
//}

bool gazebo::IAI_BoxDocking::onBoxDockRequest(sim_utils_msgs::AttachBoxTo::Request& req, sim_utils_msgs::AttachBoxTo::Response& rep)
{
  
  ROS_ERROR("BOX DOCKING REQUESTED ");
  rep.result = sim_utils_msgs::AttachBoxTo::Response::SUCCESS;//FIXME
  sim_utils_msgs::DockableRobot robot = req.robot;
  //physics::BasePtr b =  world->GetByName(robot.robot_name);FIXME i need a validity check here
  physics::ModelPtr robotModel = boost::static_pointer_cast<physics::Model>(world->GetByName(robot.robot_name));
  std::string base_link_name = tf::resolve(robot.tf_prefix, robot.base_link);
  std::string dock_link_name = tf::resolve(robot.tf_prefix, robot.dock_link);
  //physics::LinkPtr p;
  /**
  physics::Link_V links = robotModel->GetLinks();
  for (std::vector<physics::LinkPtr>::iterator lk = links.begin(); lk != links.end(); ++lk){
    physics::Collision_V cols = (*lk)->GetCollisions();
    for (std::vector<physics::CollisionPtr>::iterator cl = cols.begin(); cl != cols.end(); ++cl){
      (*cl)->SetCollideBits(BOX_COLLIDE_BIT);
    }
  }
  **/
  ROS_ERROR("Robot name %s",robot.robot_name.c_str());
  DockingPoint dp;
  math::Pose gz_box_pose = model->GetWorldPose();
  switch(req.operation){
    case sim_utils_msgs::AttachBoxTo::Request::RELEASE:
      destroyFixedLink(dock_link_name);
      //math::Pose po = model->GetWorldPose();
      gz_box_pose.pos.z -= 0.5;
      model->SetWorldPose(gz_box_pose);
      return true;
    case sim_utils_msgs::AttachBoxTo::Request::ATTACH_TOP:
      dp = BOX_DOCK_TOP;
      break;
    case sim_utils_msgs::AttachBoxTo::Request::ATTACH_BOTTOM:
      dp = BOX_DOCK_BOTTOM;
      break;
    default:
      rep.result = sim_utils_msgs::AttachBoxTo::Response::FAILURE;//FIXME
      return false;
  }
  
  
//  math::Pose gz_robot_pose_base = robotModel->GetWorldPose();
  
  std::string box_gz_link = tf::resolve(tf_prefix,"base_footprint");

  tf::StampedTransform transfom;//T_d^q
  listener.waitForTransform(box_gz_link, dock_link_name,ros::Time(0), ros::Duration(1.0));
  listener.lookupTransform(box_gz_link, dock_link_name,ros::Time(0), transfom);

  tf::StampedTransform transform_off;//T_q^q'
  //listener.waitForTransform(resolveDockName(dp), box_gz_link,ros::Time(0), ros::Duration(1.0));
  listener.lookupTransform(resolveDockName(dp), box_gz_link, ros::Time(0), transform_off);
  
  
  math::Pose transform_pose = tfToGzPose(transfom);
  math::Pose pose_box_dock = tfToGzPose(transform_off);
  
  
  math::Pose pose = pose_box_dock * transform_pose * gz_box_pose;
  
  this->model->SetWorldPose(pose);
  
  physics::LinkPtr robotLink =  robotModel->GetLink(robot.base_link);
  
  ROS_ERROR("CREATING LINK");
  return createFixedLink(dock_link_name,robotLink, true);
  ROS_ERROR("DONE");
}

bool gazebo::IAI_BoxDocking::onDockQuadRequest(sim_utils_msgs::ConnectWasp::Request& req, sim_utils_msgs::ConnectWasp::Response& rep){
  
}

bool gazebo::IAI_BoxDocking::onGeneralDockRequest(sim_utils_msgs::ConnectComponnet::Request& req, sim_utils_msgs::ConnectComponnet::Response& rep)
{
  ROS_INFO("QUADCOPTER REQUESTED DOCKING");
  
  std::string target_link = tf::resolve(req.connection.Robot1,req.connection.Robot1_Link);
  std::string source_link = tf::resolve(req.connection.Robot2,req.connection.Robot2_Link);
  
  
  std::string source_gz = tf::resolve(req.connection.Robot2,"base_link");
  
  
  physics::ModelPtr sourceModel = boost::static_pointer_cast<physics::Model>(world->GetByName(req.connection.Robot2));
  physics::ModelPtr targetModel = boost::static_pointer_cast<physics::Model>(world->GetByName(req.connection.Robot1));
  
  math::Pose pose_wasp_base = sourceModel->GetWorldPose();//T_q^w
  
  tf::StampedTransform transfom;//T_d^q
  //listener.waitForTransform(source_gz,target_link,ros::Time(0), ros::Duration(3.0));
  listener.lookupTransform(source_gz,target_link,ros::Time(0), transfom);

  tf::StampedTransform transfom_off;//T_q^q'
  //listener.waitForTransform(source_link, source_gz,ros::Time(0), ros::Duration(3.0));
  listener.lookupTransform(source_link, source_gz, ros::Time(0), transfom_off);
  
  math::Quaternion rotation = math::Quaternion( transfom.getRotation().w(), transfom.getRotation().x(), transfom.getRotation().y(), transfom.getRotation().z());
  math::Vector3 d_loc = math::Vector3(transfom.getOrigin().x(), transfom.getOrigin().y(), transfom.getOrigin().z()); //T_q^w
  math::Vector3 d_off = math::Vector3(transfom_off.getOrigin().x(), transfom_off.getOrigin().y(), transfom_off.getOrigin().z());//T_q^w
 
  math::Pose transform_pose = math::Pose(d_loc,rotation);
  math::Pose pose_box_dock = math::Pose(d_off,math::Quaternion());
  math::Pose pose = pose_box_dock * transform_pose * pose_wasp_base;
  
//  pose.pos += pose.rot * d_loc;//(d_loc-d_off);
//  pose.rot = pose.rot * rotation;
//  pose.pos += pose.rot*d_off;

  
    //FIXME something somewhere is terribly wrong
//  sourceModel->SetCollideMode("ghost");
//  sourceModel->ResetPhysicsStates();
  sourceModel->SetWorldPose(pose);
  //sourceModel->SetStatic(true);does not work
  
  
  physics::LinkPtr waspLink =  sourceModel->GetLink("base_link");
  //physics::LinkPtr boxLink = targetModel->GetLink("base_footprint");
  //FIXME
    gazebo::physics::InertialPtr in =  waspLink->GetInertial();
    in->SetMass(0.00000000000001);//FIXME find out what is going on here RESET value
    waspLink->SetInertial(in);
    waspLink->UpdateMass();
  //FIXME
  
    math::Pose tPose = targetModel->GetWorldPose();
  math::Pose offset;
//  math::Vector3 off_loc = math::Vector3(transfom.getOrigin().x(), transfom.getOrigin().y(), transfom.getOrigin().z());
  //math::Pose offset = tPose-pose;
  //targetModel->AttachStaticModel(sourceModel,offset);
  
  physics::JointPtr fixedJoint = world->GetPhysicsEngine()->CreateJoint("revolute", this->model);
  fixedJoint->Load(boxLink,
              waspLink, offset);
  fixedJoint->Init();
  fixedJoint->SetHighStop(0, 0);
  fixedJoint->SetLowStop(0, 0);
  
  
  //fixedJoint->Attach();
  fixedJoint->Attach(boxLink,waspLink);
    fixedJoint->Update();
  
  connections[source_link] = fixedJoint;
  
  return true;
}




/**
bool foo(sim_utils_msgs::ConnectComponnet::Request& req, sim_utils_msgs::ConnectComponnet::Response& rep)
{
  ROS_INFO("DOCKING REQUESTED");
  
  std::string target_link = tf::resolve(req.connection.Robot1,req.connection.Robot1_Link);
  std::string source_link = tf::resolve(req.connection.Robot2,req.connection.Robot2_Link);
  
  
  std::string source_gz = tf::resolve(req.connection.Robot2,"base_link");
  
  
  physics::ModelPtr sourceModel = boost::static_pointer_cast<physics::Model>(world->GetByName(req.connection.Robot2));
  physics::ModelPtr targetModel = boost::static_pointer_cast<physics::Model>(world->GetByName(req.connection.Robot1));
  
  math::Pose pose = sourceModel->GetWorldPose();
  
  tf::StampedTransform transfom;
  listener.lookupTransform(source_gz,target_link,ros::Time(0), transfom);

  tf::StampedTransform transfom_off;
  listener.lookupTransform(source_link, source_gz, ros::Time(0), transfom_off);
  
  math::Quaternion rotation = math::Quaternion( transfom.getRotation().w(), transfom.getRotation().x(), transfom.getRotation().y(), transfom.getRotation().z());
  math::Vector3 d_loc = math::Vector3(transfom.getOrigin().x(), transfom.getOrigin().y(), transfom.getOrigin().z());  
  math::Vector3 d_off = math::Vector3(transfom_off.getOrigin().x(), transfom_off.getOrigin().y(), transfom_off.getOrigin().z());
  
  pose.pos += pose.rot * d_loc;//(d_loc-d_off);
  pose.rot = pose.rot * rotation;
  pose.pos += pose.rot*d_off;

  sourceModel->SetWorldPose(pose);
  //sourceModel->SetStatic(true);does not work



  
  

  
  physics::LinkPtr waspLink =  sourceModel->GetLink("base_link");
  physics::LinkPtr boxLink = targetModel->GetLink("base_footprint");
  
  math::Pose tPose = targetModel->GetWorldPose();
  math::Pose offset;
//  math::Vector3 off_loc = math::Vector3(transfom.getOrigin().x(), transfom.getOrigin().y(), transfom.getOrigin().z());
  //math::Pose offset = tPose-pose;
  //targetModel->AttachStaticModel(sourceModel,offset);
  calendar
  physics::JointPtr fixedJoint = world->GetPhysicsEngine()->CreateJoint("revolute", this->model);
  fixedJoint->Load(boxLink,
              waspLink, offset);
  fixedJoint->Init();
  fixedJoint->SetHighStop(0, 0);
  fixedJoint->SetLowStop(0, 0);
  
  
  //fixedJoint->Attach();
  fixedJoint->Attach(boxLink,waspLink);
  
  connections[source_link] = fixedJoint;
  
  return true;
}
**/
void gazebo::IAI_BoxDocking::onUpdate()
{
  ros::Time currentTime = ros::Time::now();
  double d_time = 
        (currentTime - lastUpdate).toSec();
  if(d_time < updateRate){
    return;
  }
  math::Pose pose = this->model->GetWorldPose();
  tf::Quaternion qt(pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w);
  tf::Vector3 vt(pose.pos.x, pose.pos.y, pose.pos.z);

  tf::Transform transform(qt, vt);
    
  tf_broadcaster->sendTransform(
    tf::StampedTransform(transform, ros::Time::now(), fixed_frame,
	base_frame));
  lastUpdate = currentTime;
}

