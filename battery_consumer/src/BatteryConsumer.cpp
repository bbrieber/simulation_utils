#include "battery_consumer/BatteryConsumer.hpp"

namespace gazebo 
{
BatteryConsumer::BatteryConsumer(): ModelPlugin(),
    default_consumer(DEFAULT_CONSUMER_NAME, DEFAULT_CONSUMER_CONSUME),
    max_battery_state(DEFAULT_MAX_CHARGE),
    charging(false),
    battery_state(DEFAULT_MAX_CHARGE),
    update_rate(1.0),
    charge(0),
    dicharge_amount(1.0)
    {
    }

void BatteryConsumer::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  if (!ros::isInitialized())
      {
	ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
	return;
      }
      double default_consume = DEFAULT_CONSUMER_CONSUME;
      std::string name = DEFAULT_CONSUMER_NAME;
      if (_sdf->HasElement("robotNamespace")) namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
      if (_sdf->HasElement("default_consume")) default_consume = _sdf->GetElement("default_consume")->Get<double>();
      if (_sdf->HasElement("default_consumer_name")) name = _sdf->GetElement("default_consumer_name")->Get<std::string>();
      
      
      default_consumer.first = name;
      default_consumer.second = default_consume;
      consumers.insert(default_consumer);
      
      
      this->model = _parent;
      n = ros::NodeHandle(namespace_);
      
      
      battery_pub = n.advertise<std_msgs::Float64>("battery_state", 1);
      
      add_consumer_service = n.advertiseService("add_consumer",&BatteryConsumer::addConsumer,this);
      
      
      remove_consumer_service = n.advertiseService("remove_consumer",&BatteryConsumer::removeConsumer,this);
      last_time = model->GetWorld()->GetSimTime().Double();
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&BatteryConsumer::OnUpdate, this, _1));
}

void BatteryConsumer::OnUpdate(const common::UpdateInfo&)
{
      current_time = model->GetWorld()->GetSimTime().Double();
      double time_diff = current_time-last_time; 
      if(update_rate< time_diff){
	if(charging){
	  //TODO recharge stuff
	}else{
	  dicharge_amount = 0;
	  for( std::map<std::string, double>::iterator ii=consumers.begin(); ii!=consumers.end(); ++ii)
	  {
	    dicharge_amount +=  (*ii).second;
	    
	  }
	  
	  battery_state -= time_diff*  dicharge_amount;
	  if(battery_state<0){
	    battery_state==0;
	  }
	  
	  //std::cout << "battery_state: " << battery_state << std::endl;

	}
	std_msgs::Float64 msg;
	msg.data = battery_state;
	battery_pub.publish<std_msgs::Float64>(msg);
	
	last_time = current_time;
      }

}


bool BatteryConsumer::addConsumer(sim_utils_msgs::AddBatteryConsumerRequest& req, sim_utils_msgs::AddBatteryConsumerResponse& res)
{

	std::map<std::string,double>::iterator it = consumers.find(req.name);
	if(it == consumers.end())
	{
	  consumers[req.name] = req.amount;
	  res.ret_value = sim_utils_msgs::AddBatteryConsumer::Response::OK;
	}
	else{
	  res.ret_value = sim_utils_msgs::AddBatteryConsumer::Response::CONSUMER_NAME_CLASH;
	}
	return true;
}

bool BatteryConsumer::removeConsumer(sim_utils_msgs::RemoveBatteryConsumerRequest& req, sim_utils_msgs::RemoveBatteryConsumerResponse& res)
{

	std::map<std::string,double>::iterator it = consumers.find(req.name);
	if(it == consumers.end())
	{
	  res.ret_value = sim_utils_msgs::RemoveBatteryConsumer::Response::NO_SUCH_CONSUMER;
	}
	else{
	  consumers.erase(it);
	  res.ret_value = sim_utils_msgs::RemoveBatteryConsumer::Response::OK;
	}
}

  
}
