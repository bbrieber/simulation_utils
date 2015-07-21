#ifndef __IAI_GAZEBO_OPERATOR_MODEL__
#define __IAI_GAZEBO_OPERATOR_MODEL__

#include "gazebo_mission_control/OperatorDefs.hpp"

#include <boost/bind.hpp>
#include <gazebo-5.1/gazebo/gazebo.hh>
#include <gazebo-5.1/gazebo/physics/physics.hh>
#include <gazebo-5.1/gazebo/common/common.hh>
#include <stdio.h>
#include <gazebo-5.1/gazebo/physics/PhysicsTypes.hh>
#include <gazebo-5.1/gazebo/common/CommonTypes.hh>

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>

namespace gazebo
{
  class OperatorModelPlugin : public ModelPlugin
 { 
   public:
    OperatorModelPlugin();
    ~OperatorModelPlugin();

    void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);
    void onUpdate();
    void onTwist(const geometry_msgs::Twist::ConstPtr& msg);
    //void onChangeAction(const std_msgs::String::ConstPtr& msg);
    void onPause();
    
  private:
    
    physics::ModelPtr model;   
    event::ConnectionPtr updateConnection;
    event::ConnectionPtr pauseConnection;
    ros::NodeHandle *nh;
    ros::Subscriber twistSub;
    
    geometry_msgs::Twist currentTwist;
    void init_ros();
    
    double current_rotation;
    double speed;
 };
 
 GZ_REGISTER_MODEL_PLUGIN(OperatorModelPlugin);
}
  
#endif