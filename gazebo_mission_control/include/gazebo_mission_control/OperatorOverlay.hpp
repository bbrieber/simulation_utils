#ifndef __IAI_GAZEBO_OPERATOR_OVERLAY__
#define __IAI_GAZEBO_OPERATOR_OVERLAY__




#include "gazebo/gazebo.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/gui/GuiPlugin.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/gui/gui.hh"


#include <gazebo/gui/MouseEventHandler.hh>
#include <gazebo/common/MouseEvent.hh>


#include <QVBoxLayout>

namespace gazebo
{
    class GAZEBO_VISIBLE OperatorOverlay : public GUIPlugin
    {
      Q_OBJECT

      /// \brief Constructor
      /// \param[in] _parent Parent widget
      public: 
        OperatorOverlay();
        /// \brief Destructor
        virtual ~OperatorOverlay();
        bool onMouseClick(const common::MouseEvent & _event);
        bool onMouseMove(const common::MouseEvent & _event);
        void Load(int /*_argc*/, char ** /*_argv*/);
	void selectionChanged(ConstSelectionPtr &sel);
	void onUpdate(const common::UpdateInfo & /*_info*/);
	void mouseMoveEvent(QMouseEvent *ev);
     // protected slots: void OnButton();
	protected slots: 
	  void setTarget();
	  void moveOperator();
      

      private: 
	
	unsigned int counter;
	transport::SubscriberPtr selectionSub;
	transport::NodePtr node;
	transport::PublisherPtr factoryPub;
	void init();
	std::string current_selection;
	std::string current_target;
	
	bool move_to_mode;
	QLineEdit *targetLabel;
	
	event::ConnectionPtr updateConnection;
    };
    
}

#endif