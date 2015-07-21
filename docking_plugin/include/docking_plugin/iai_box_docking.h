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

#ifndef IAI_BOX_DOCKING_H
#define IAI_BOX_DOCKING_H

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <unordered_map>


#include <sim_utils_msgs/ConnectComponnet.h>
#include <sim_utils_msgs/AttachBoxTo.h>
#include <sim_utils_msgs/ConnectWasp.h>


#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#define BOX_COLLIDE_BIT 0x80
#define DEFAULT_COLLIDE_BIT 0x01
// #define BOX_ANTICOLLIDE_BIT 0x00

namespace gazebo
{
  
  typedef enum { 
    QUAD_FRONT_LEFT,
    QUAD_FRONT_RIGHT,
    QUAD_BACK_LEFT,
    QUAD_BACK_RIGHT,
    BOX_DOCK_TOP,
    BOX_DOCK_BOTTOM,
    GRIPPER_DOCK_FRONT,
    GRIPPER_DOCK_TOP
  }DockingPoint;

  class IAI_BoxDocking : public ModelPlugin
  {
  public:
      void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
      bool onGeneralDockRequest(sim_utils_msgs::ConnectComponnet::Request &req, sim_utils_msgs::ConnectComponnet::Response &rep);
      bool onDockQuadRequest(sim_utils_msgs::ConnectWasp::Request &req, sim_utils_msgs::ConnectWasp::Response &rep);
      bool onBoxDockRequest(sim_utils_msgs::AttachBoxTo::Request &req, sim_utils_msgs::AttachBoxTo::Response &rep);
      void onUpdate();
      void updateCollisionMask(bool enabled);

  private: 
    
      std::string resolveDockName(DockingPoint dp);
      math::Pose tfToGzPose(tf::StampedTransform transform);
      
      bool createFixedLink(std::string name, physics::LinkPtr link, bool setAsParent);
      bool destroyFixedLink(std::string name);
      
      float updateRate;
      ros::Time lastUpdate;
      
      ros::ServiceServer attachRequestService;
      ros::ServiceServer waspDockRequestService;
      ros::ServiceServer genralDockRequestService;
      ros::NodeHandle *n;
      
      boost::shared_ptr<tf::TransformBroadcaster> tf_broadcaster;
      tf::TransformListener listener;
      physics::LinkPtr boxLink;
      
      std::string tf_prefix;
      std::string base_frame;
      std::string fixed_frame;
      std::string robot_namespace_;
      
      
      std::map<std::string, physics::JointPtr> connections;      
      std::unordered_map< DockingPoint, std::string, std::hash<int> > dockingPoints;
      std::unordered_map< DockingPoint, std::string, std::hash<int> > dockedComponents;
      
      event::ConnectionPtr updateConnection;
      physics::WorldPtr world;
      physics::ModelPtr model;
  };
  
  GZ_REGISTER_MODEL_PLUGIN(IAI_BoxDocking)
}
#endif // IAI_BOX_DOCKING_H
