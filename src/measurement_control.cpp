/*
 * Brad Fletcher
 * brad@udel.edu
 * Thanks to Georgia Tech for allowing me to use their 
 * RViz panel code as a model for this.
 */


#include "measurement_control.h"
#include "FlowLayout.h"

//added
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ud_measurement_panel/MeasurementCommand.h"
#include <sstream>

namespace ud_measurement_space
{

MeasurementPanel::MeasurementPanel(QWidget *parent)
    : rviz::Panel(parent)
{
    content = new MeasurementControlTab;
    QHBoxLayout* panelLayout = new QHBoxLayout;
    panelLayout->addWidget(content);
    setLayout(panelLayout);
}

MeasurementControlTab::~MeasurementControlTab()
{
    refreshManager->alive = false;
    refreshManager->quit();
    refreshManager->wait();
}

MeasurementControlTab::MeasurementControlTab(QWidget *parent)
    : QTabWidget(parent)
{

    groupStyleSheet = "QGroupBox {"
                     "border: 1px solid gray;"
                      "border-radius: 9px;"
                      "margin-top: 0.5em;"
                      "}"
                      "QGroupBox::title {"
                      "subcontrol-origin: margin;"
                      "left: 10px;"
                      "padding: 0 3px 0 3px;"
                      "}";
                     
    //added ros stuff
    int argc = 0;
    char **argv;

    ros::init(argc, argv, "talker");
    measurement_pub = n.advertise<ud_measurement_panel::MeasurementCommand>("measurement_command", 1000);
    //

    //Load Tabs
    
    initializeToolTab();
    std::cerr << "Tool Tab Loaded" << std::endl;

    addTab(toolTab, "Measurement");

    
    refreshManager = new RVizRefreshManager;
    refreshManager->parentWidget = this;
    connect(this, SIGNAL(sendWaitTime(int)), refreshManager, SLOT(getWaitTime(int)));
    refreshManager->start();
}

void MeasurementControlTab::initializeToolTab()
{
       
    //Create a box layout.

  QVBoxLayout* toolStateLayout = new QVBoxLayout;
  
  //Create a group box for "Removing Points" action

  QGroupBox* deleteBox = new QGroupBox;
  deleteBox->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  deleteBox->setAlignment(Qt::AlignHCenter | Qt::AlignTop);
  deleteBox->setStyleSheet(groupStyleSheet);
  deleteBox->setTitle("Delete...");
  
  // Delete buttons layout
  QGridLayout* deleteLayout = new QGridLayout;
  deleteLayout->setAlignment(Qt::AlignHCenter | Qt::AlignTop);
  
  // Delete Last Point Button
  QPushButton* btnDeleteSelected = new QPushButton;
  btnDeleteSelected->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  btnDeleteSelected->setText("Selected point");
  deleteLayout->addWidget(btnDeleteSelected, 0, 1, 1, 1, Qt::AlignHCenter | Qt::AlignVCenter);
  
  // Connect Button
  connect(btnDeleteSelected, SIGNAL(clicked()), this, SLOT(btnDeleteSelectedClick()));

  // Delete All Points Button
  QPushButton* btnDeleteAllPoints = new QPushButton;
  btnDeleteAllPoints->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  btnDeleteAllPoints->setText("All points");
  deleteLayout->addWidget(btnDeleteAllPoints, 0, 0, 1, 1, Qt::AlignHCenter | Qt::AlignVCenter);
  
  // Connect Button
  connect(btnDeleteAllPoints, SIGNAL(clicked()), this, SLOT(btnDeleteAllClick()));
    
  
  // Connect Box and Layout
  deleteBox->setLayout(deleteLayout);
  
  ///////////  End of Removing Points //////////////////
  
  //Create a group box for "Estimation" action

  QGroupBox* estimationBox = new QGroupBox;
  estimationBox->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  estimationBox->setAlignment(Qt::AlignHCenter | Qt::AlignTop);
  estimationBox->setStyleSheet(groupStyleSheet);
  estimationBox->setTitle("Estimate...");
  
  // Estimate buttons layout
  QGridLayout* estimateLayout = new QGridLayout;
  estimateLayout->setAlignment(Qt::AlignHCenter | Qt::AlignTop);
  
  // Estimate Line Button
  QPushButton* btnEstimateLine = new QPushButton;
  btnEstimateLine->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  btnEstimateLine->setText("Line");
  estimateLayout->addWidget(btnEstimateLine, 0, 0, 1, 1, Qt::AlignHCenter | Qt::AlignVCenter);
  
  // Connect Button
  connect(btnEstimateLine, SIGNAL(clicked()), this, SLOT(btnEstimateLineClick()));
    
  // Estimate Plane Button
  QPushButton* btnEstimatePlane = new QPushButton;
  btnEstimatePlane->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  btnEstimatePlane->setText("Plane");
  estimateLayout->addWidget(btnEstimatePlane, 0, 1, 1, 1, Qt::AlignHCenter | Qt::AlignVCenter);
  
  // Connect Button
  connect(btnEstimatePlane, SIGNAL(clicked()), this, SLOT(btnEstimatePlaneClick()));

  // Estimate Circle Button
  QPushButton* btnEstimateCircle = new QPushButton;
  btnEstimateCircle->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  btnEstimateCircle->setText("Circle");
  estimateLayout->addWidget(btnEstimateCircle, 1, 0, 1, 1, Qt::AlignHCenter | Qt::AlignVCenter);
  
  // Connect Button
  connect(btnEstimateCircle, SIGNAL(clicked()), this, SLOT(btnEstimateCircleClick()));

  // Estimate Rigid Transform Button
  QPushButton* btnEstimateRigidTransform = new QPushButton;
  btnEstimateRigidTransform->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  btnEstimateRigidTransform->setText("Rigid transform");
  estimateLayout->addWidget(btnEstimateRigidTransform, 1, 1, 1, 1, Qt::AlignHCenter | Qt::AlignVCenter);
  
  // Connect Button
  connect(btnEstimateRigidTransform, SIGNAL(clicked()), this, SLOT(btnEstimateRigidTransformClick()));
  
  // Connect Box and Layout
  estimationBox->setLayout(estimateLayout);
  
  ////////// End of Estimation ///////////////
  
    //Create a group box for "Measure" action

  QGroupBox* measureBox = new QGroupBox;
  measureBox->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  measureBox->setAlignment(Qt::AlignHCenter | Qt::AlignTop);
  measureBox->setStyleSheet(groupStyleSheet);
  measureBox->setTitle("Measure...");
  
  // Measure buttons layout
  QGridLayout* measureLayout = new QGridLayout;
  measureLayout->setAlignment(Qt::AlignHCenter | Qt::AlignTop);
  
  // Measure Line Button
  QPushButton* btnMeasureLine = new QPushButton;
  btnMeasureLine->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  btnMeasureLine->setText("Length");
  measureLayout->addWidget(btnMeasureLine, 0, 0, 1, 1, Qt::AlignHCenter | Qt::AlignVCenter);
  
  // Connect Button
  connect(btnMeasureLine, SIGNAL(clicked()), this, SLOT(btnMeasureLengthClick()));
  
  // Connect Box and Layout
  measureBox->setLayout(measureLayout);
  
  ////////// End of Measure ///////////////
  
  //Create a group box for "Crop" action

  QGroupBox* cropBox = new QGroupBox;
  cropBox->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  cropBox->setAlignment(Qt::AlignHCenter | Qt::AlignTop);
  cropBox->setStyleSheet(groupStyleSheet);
  cropBox->setTitle("Crop");
  
  // Estimate buttons layout
  QGridLayout* cropLayout = new QGridLayout;
  cropLayout->setAlignment(Qt::AlignHCenter | Qt::AlignTop);
  
  // Crop Button
  QPushButton* btnCrop = new QPushButton;
  btnCrop->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  btnCrop->setText("Crop");
  cropLayout->addWidget(btnCrop, 0, 0, 1, 1, Qt::AlignHCenter | Qt::AlignVCenter);
  
  // Connect Button
  connect(btnCrop, SIGNAL(clicked()), this, SLOT(btnCropClick()));
    
  // Undo Button
  QPushButton* btnUndo = new QPushButton;
  btnUndo->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  btnUndo->setText("Undo");
  cropLayout->addWidget(btnUndo, 0, 1, 1, 1, Qt::AlignHCenter | Qt::AlignVCenter);
  
   // Connect Button
  connect(btnUndo, SIGNAL(clicked()), this, SLOT(btnUndoClick()));
    
  // Connect Box and Layout
  cropBox->setLayout(cropLayout);
  
  ////////// End of Crop ///////////////
  
  //Will need to connect this later

  toolStateLayout->addWidget(deleteBox, Qt::AlignHCenter | Qt::AlignTop);
  toolStateLayout->addWidget(estimationBox, Qt::AlignHCenter | Qt::AlignTop);
  toolStateLayout->addWidget(measureBox, Qt::AlignHCenter | Qt::AlignTop);
  toolStateLayout->addWidget(cropBox, Qt::AlignHCenter | Qt::AlignTop);
  
  //Create the tab
  toolTab = new QWidget;
  //Set the tab's master layout
  toolTab->setLayout(toolStateLayout);
    
}

void RVizRefreshManager::run()
{
    alive = true;
    waitTime = 1000;
    connect(this, SIGNAL(signalRefresh()), parentWidget, SLOT(refreshState()));
    while(alive)
    {
        emit signalRefresh();
        this->msleep(waitTime);
    }
    emit finished();
}

void RVizRefreshManager::getWaitTime(int t)
{
    waitTime = t;
}


void MeasurementPanel::save(rviz::Config config) const
{
    rviz::Panel::save(config);

    /*
    config.mapSetValue("Class", getClassId());

    rviz::Config ip_config = config.mapMakeChild("HuboIP");

    QVariant a = QVariant(content->getIPAddress(0));
    QVariant b = QVariant(content->getIPAddress(1));
    QVariant c = QVariant(content->getIPAddress(2));
    QVariant d = QVariant(content->getIPAddress(3));

    ip_config.mapSetValue("ipAddrA", a);
    ip_config.mapSetValue("ipAddrB", b);
    ip_config.mapSetValue("ipAddrC", c);
    ip_config.mapSetValue("ipAddrD", d);
*/

}

void MeasurementPanel::load(const rviz::Config &config)
{
    rviz::Panel::load(config);

    /*
    rviz::Config ip_config = config.mapGetChild("HuboIP");
    QVariant a, b, c, d;
    if( !ip_config.mapGetValue("ipAddrA", &a) || !ip_config.mapGetValue("ipAddrB", &b)
     || !ip_config.mapGetValue("ipAddrC", &c) || !ip_config.mapGetValue("ipAddrD", &d))
        ROS_INFO("Loading the IP Address Failed");
    else
        content->setIPAddress(a.toInt(), b.toInt(), c.toInt(), d.toInt()); */
}


} // ud_measurement_space




#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( ud_measurement_space::MeasurementPanel,rviz::Panel )
PLUGINLIB_EXPORT_CLASS( ud_measurement_space::MeasurementControlTab, QTabWidget )
PLUGINLIB_EXPORT_CLASS( ud_measurement_space::RVizRefreshManager, QThread )
