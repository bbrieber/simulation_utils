/*
 * Copyright (C) 2014 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include <sstream>
#include <gazebo/msgs/msgs.hh>
#include "gazebo_mission_control/OperatorOverlay.hpp"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_GUI_PLUGIN(OperatorOverlay)

/////////////////////////////////////////////////
OperatorOverlay::OperatorOverlay()
  : GUIPlugin()
{
  init();
  this->counter = 0;

  // Set the frame background and foreground colors
  this->setStyleSheet(
      "QFrame { background-color : rgba(100, 100, 100, 255); color : white; }");

  // Create the main layout
  QHBoxLayout *mainLayout = new QHBoxLayout;

  // Create the frame to hold all the widgets
  QFrame *mainFrame = new QFrame();

  // Create the layout that sits inside the frame
  QVBoxLayout *frameLayout = new QVBoxLayout();

    // ACTIONS chooser
  QLabel *operatorLabel = new QLabel(tr("Operator:"));
  frameLayout->addWidget(operatorLabel); 
  
  
  QPushButton *setTargetButton = new QPushButton(tr("Selection to Target"));
  connect(setTargetButton, SIGNAL(clicked()), this, SLOT(setTarget()));
  frameLayout->addWidget(setTargetButton);
  
  targetLabel = new QLineEdit(tr("target_name"));
  targetLabel->setEnabled(false);
  frameLayout->addWidget(targetLabel);
  
  QPushButton *MoveToButton = new QPushButton( "Move To");
  MoveToButton->setCheckable( true );
  connect(MoveToButton, SIGNAL(clicked()), this, SLOT(moveOperator()));
  frameLayout->addWidget(MoveToButton);

  
  // ROBOT chooser
  QLabel *robotLabel = new QLabel(tr("Robot:"));
  frameLayout->addWidget(robotLabel);
  
  QComboBox *robotChooser = new QComboBox();
  robotChooser->addItem("ALL");
  robotChooser->addItem("A");
  robotChooser->addItem("Donkey");
  robotChooser->addItem("Hawk");
  robotChooser->addItem("Wasp1");
  robotChooser->addItem("Wasp2");
  robotChooser->addItem("Wasp3");
  robotChooser->addItem("Wasp4");
  ///FIXME add SIGNAL
  frameLayout->addWidget(robotChooser);


  // ACTIONS chooser
  QLabel *actionLabel = new QLabel(tr("Commands:"));
  frameLayout->addWidget(actionLabel);
  
  QComboBox *actionChooser = new QComboBox();
  actionChooser->addItem("Perceive");
  actionChooser->addItem("Search");
  actionChooser->addItem("Recharge");
  actionChooser->addItem("Drop box");
  actionChooser->addItem("Pick up box");
  actionChooser->addItem("Follow e");
  actionChooser->addItem("Come Here");

  frameLayout->addWidget(actionChooser);

  
  QPushButton *executeAction = new QPushButton(tr("Execute"));
  //connect(button2, SIGNAL(clicked()), this, SLOT(OnButton()));
  frameLayout->addWidget(executeAction);

  
  ///LOGGING
  QLabel *logLayout = new QLabel("Logging:");
  frameLayout->addWidget(logLayout);  
  QPushButton *logButton = new QPushButton( "Logging!");
  logButton->setCheckable( true );
  frameLayout->addWidget(logButton);

  QPushButton *exportButton = new QPushButton(tr("Export log"));
  //connect(button2, SIGNAL(clicked()), this, SLOT(OnButton()));
  frameLayout->addWidget(exportButton);
  
  
  
  // Add frameLayout to the frame
  mainFrame->setLayout(frameLayout);

  // Add the frame to the main layout
  mainLayout->addWidget(mainFrame);

  // Remove margins to reduce space
  frameLayout->setContentsMargins(0, 0, 0, 0);
  mainLayout->setContentsMargins(0, 0, 0, 0);

  this->setLayout(mainLayout);

  // Position and resize this widget
  this->move(10, 10);
  this->resize(120, 300);

  // Create a node for transportation
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  selectionSub =  this->node->Subscribe("~/selection", & OperatorOverlay::selectionChanged, this);
  this->factoryPub = this->node->Advertise<msgs::Factory>("~/factory");
  this->setMouseTracking(true);
  
  
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&OperatorOverlay::onUpdate, this, _1));
}

void OperatorOverlay::onUpdate(const common::UpdateInfo & /*_info*/)
{
  QPoint p = this->mapFromGlobal(QCursor::pos());
  
  std::cout  << "x: "<<p.x() << "y: " << p.y();
  if(move_to_mode){
        QPoint p = this->mapFromGlobal(QCursor::pos());
  }
}

void OperatorOverlay::selectionChanged(ConstSelectionPtr &sel)
{
  std::cout  << sel->DebugString();
  this->current_selection = sel->name();
 
}

void OperatorOverlay::setTarget()
{
  this->current_target = this->current_selection;
  std::cout  << "changing target "<<current_target;
  targetLabel->setText(current_target.c_str());
}

/////////////////////////////////////////////////
OperatorOverlay::~OperatorOverlay()
{
}
bool OperatorOverlay::onMouseClick(const common::MouseEvent & _event)
{
  std::cout  << "click";
    return true;
}
bool OperatorOverlay::onMouseMove(const common::MouseEvent & _event)
{
  std::cout  << "click";
    return true;
}

void OperatorOverlay::mouseMoveEvent(QMouseEvent *ev) {
        // vvv That's where the magic happens
    std::cout  << "move";
    }

void OperatorOverlay::init()
{   
///std::cout  << "#####################LOAD#####################";
    move_to_mode = false;
  
    gui::MouseEventHandler::Instance()->AddPressFilter("glwidget",
    boost::bind(&OperatorOverlay::onMouseClick, this, _1));
    gui::MouseEventHandler::Instance()->AddMoveFilter("glwidget",
    boost::bind(&OperatorOverlay::onMouseMove, this, _1));
}


    // Called after the plugin has been constructed.
void OperatorOverlay::Load(int /*_argc*/, char ** /*_argv*/)
{   
    std::cout  << "LOAD";
    move_to_mode = false;
  
    gui::MouseEventHandler::Instance()->AddPressFilter("glwidget",
    boost::bind(&OperatorOverlay::onMouseClick, this, _1));
    gui::MouseEventHandler::Instance()->AddPressFilter("FOOOOO",
    boost::bind(&OperatorOverlay::onMouseClick, this, _1));
    gui::MouseEventHandler::Instance()->AddMoveFilter("glwidget",
    boost::bind(&OperatorOverlay::onMouseMove, this, _1));
    
}


void OperatorOverlay::moveOperator()
{
  
   this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&OperatorOverlay::onUpdate, this, _1));
}