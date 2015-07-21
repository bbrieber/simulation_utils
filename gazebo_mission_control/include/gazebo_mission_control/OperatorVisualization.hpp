#ifndef __IAI_GAZEBO_OPERATOR_VISUAL__
#define __IAI_GAZEBO_OPERATOR_VISUAL__




#include "gazebo/gazebo.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/rendering/rendering.hh"

#include <map>

#include "ros/ros.h"

namespace gazebo
{
    class OperatorVisualization : public SystemPlugin
    {
      public: 
	~OperatorVisualization();
	OperatorVisualization();
	void onUpdate();
	void Load(int /*_argc*/, char ** /*_argv*/);
	bool onMouseClick(const common::MouseEvent & _event);
	bool onMouseMove(const common::MouseEvent & _event);
      private: 
	std::vector<event::ConnectionPtr> connections;
	event::ConnectionPtr updateConnection;
	event::ConnectionPtr pauseConnection;
	
	Ogre::Camera *cam;
	rendering::UserCameraPtr userCam;
	
	
	Ogre::Root *root;
	Ogre::SceneManager *sceneMgr;
	Ogre::SceneNode *node;
	
	Ogre::AnimationState* currentState;
	std::map<std::string, Ogre::AnimationState*> animationStates;
	
	float animationState;
	
	
    };
    
    GZ_REGISTER_SYSTEM_PLUGIN(OperatorVisualization);
}

#endif