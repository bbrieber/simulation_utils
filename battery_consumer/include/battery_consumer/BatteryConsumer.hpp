#ifndef __IAI_BATTERY_CONSUMER_HPP__
#define __IAI_BATTERY_CONSUMER_HPP__

#include "sim_utils_msgs/AddBatteryConsumer.h"
#include "sim_utils_msgs/RemoveBatteryConsumer.h"
#include "sim_utils_msgs/RechargeBatteryAction.h"

#include <stdio.h>
#include <map>
#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <ros/ros.h>

#include "std_msgs/Float64.h"


#define DEFAULT_CONSUMER_NAME "default_consumer"
#define DEFAULT_CONSUMER_CONSUME 1 //FIXME THIS IS FAR TO HIGH but we want to the triggers for testing
#define DEFAULT_MAX_CHARGE 100.


namespace gazebo
{
  class BatteryConsumer : public ModelPlugin
  {
  public:
    BatteryConsumer();
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
    void OnUpdate(const common::UpdateInfo & /*_info*/);
    bool addConsumer(sim_utils_msgs::AddBatteryConsumerRequest &req, sim_utils_msgs::AddBatteryConsumerResponse &res);
    bool removeConsumer(sim_utils_msgs::RemoveBatteryConsumerRequest &req, sim_utils_msgs::RemoveBatteryConsumerResponse &res);
    //void rechargeCB(const sim_utils_msgs::RechargeBatteryActionGoalConstPtr &goal);
    
  private:
    physics::ModelPtr model;
    event::ConnectionPtr updateConnection;
    double battery_state;
    double max_battery_state;
    double charge;//
    bool charging;
  
    double update_rate;
    std::map<std::string, double> consumers;      
    std::pair<std::string, double> default_consumer; 
    
    double dicharge_amount;
    
    double current_time;
    double last_time;
    ros::NodeHandle n;
    ros::Publisher battery_pub;
    ros::ServiceServer add_consumer_service;
    ros::ServiceServer remove_consumer_service;
    //actionlib::SimpleActionServer<sim_utils_msgs::RechargeBatteryAction> *rechargeActionServer; 
    sim_utils_msgs::RechargeBatteryFeedback rechargefeedback;
    sim_utils_msgs::RechargeBatteryResult rechargeResult;
    
    std::string namespace_;
  };
  GZ_REGISTER_MODEL_PLUGIN(BatteryConsumer)
}
#endif