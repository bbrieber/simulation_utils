#include "gazebo_mission_control/OperatorVisualization.hpp"
#include <ros/package.h>
#include <gazebo/gui/MouseEventHandler.hh>

namespace gazebo{
  OperatorVisualization::OperatorVisualization()
  {

  }

  OperatorVisualization::~OperatorVisualization()
  {

  }


  void OperatorVisualization::onUpdate()
  {

  }


  void OperatorVisualization::Load(int argc, char ** argv){    
    
    std::string path = ros::package::getPath("iai_rescue_operator_gazebo");
    std::string resource = path+"/models/sherpa_operator/";
    std::cerr<< path << std::endl;
    
    gui::MouseEventHandler::Instance()->AddPressFilter("glwidget",boost::bind(&OperatorVisualization::onMouseClick, this, _1));
    //gui::MouseEventHandler::Instance()->AddMoveFilter("glwidget",   boost::bind(&OperatorVisualization::onMouseMove, this, _1));
    
  }
  
  bool OperatorVisualization::onMouseClick(const common::MouseEvent & _event)
  {
      std::cout  << "click";
      gui::MouseEventHandler::Instance()->RemoveMoveFilter("glwidget");
      
      return true;
  }
  
  bool OperatorVisualization::onMouseMove(const common::MouseEvent & _event)
  {
    std::cout  << "mouse";
      return false;
  }


}