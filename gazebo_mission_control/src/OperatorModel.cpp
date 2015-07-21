#include "gazebo_mission_control/OperatorModel.hpp"

/**
 * TODO :
 *  make rosconnection configurable
 *  make main frame configurable
 *  make speed configurable
 */

gazebo::OperatorModelPlugin::OperatorModelPlugin():
current_rotation(0.0),
speed(1.0)
{
  
}


gazebo::OperatorModelPlugin::~OperatorModelPlugin()
{

}


void gazebo::OperatorModelPlugin::Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr)
{

    this->model = _parent;
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                                 boost::bind(&OperatorModelPlugin::onUpdate, this));
    
    init_ros();
}


void gazebo::OperatorModelPlugin::onPause()
{

}


void gazebo::OperatorModelPlugin::onTwist(const geometry_msgs::Twist::ConstPtr& msg)
{
  currentTwist.angular = msg->angular;
  currentTwist.angular.x = 0.0;//Enforce 0.0 rotation
  currentTwist.angular.y = 0.0;//Enforce 0.0 rotation
  currentTwist.linear = msg->linear;
  currentTwist.linear.z = -9.81;
}


void gazebo::OperatorModelPlugin::onUpdate()
{
  
      //TODO i think i should just set the worldpose instead of using vel...
      math::Pose p = model->GetWorldPose();
      p.rot.x = 0.0;
      p.rot.y = 0.0;
      model->SetWorldPose(p);
      double stepTime = this->model->GetWorld()->GetPhysicsEngine()->GetMaxStepSize();
      this->current_rotation += currentTwist.angular.z*stepTime;
      
      
      
      float sn = sin(current_rotation);
      float cs = cos(current_rotation);
      
      model->SetAngularVel(math::Vector3(currentTwist.angular.x,currentTwist.angular.y,currentTwist.angular.z));
      model->SetLinearVel(math::Vector3((currentTwist.linear.x*cs - currentTwist.linear.y *sn)  * this->speed,(currentTwist.linear.x*sn + currentTwist.linear.y *cs)  * this->speed,currentTwist.linear.z));
      
      
}


void gazebo::OperatorModelPlugin::init_ros()
{
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }
    
    nh = new ros::NodeHandle("Operator");
    twistSub = nh->subscribe<geometry_msgs::Twist>("/cmd_vel", 1, &OperatorModelPlugin::onTwist,this);
}
