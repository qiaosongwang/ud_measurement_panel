/*
 * Brad Fletcher
 * brad@udel.edu
 * Thanks to Georgia Tech for allowing me to use their 
 * RViz panel code as a model for this.
 */

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------

#include "measurement_control.h"
#include "FlowLayout.h"

//added
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ud_measurement_panel/MeasurementCommand.h"
#include <sstream>

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------

namespace ud_measurement_space
{

//----------------------------------------------------------------------------

MeasurementPanel::MeasurementPanel(QWidget *parent) : rviz::Panel(parent)
{

  //  QHBoxLayout *panelLayout = new QHBoxLayout;
  QVBoxLayout *panelLayout = new QVBoxLayout;
  //  QGridLayout *panelLayout = new QGridLayout;

  controlTabWidget = new MeasurementControlTab();
  //  controlTabWidget->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  
  //  panelLayout->addWidget(deleteBox, Qt::AlignHCenter | Qt::AlignTop);
  panelLayout->addWidget(controlTabWidget);
  //  panelLayout->addWidget(controlTabWidget, 1, 0, 3, 3, Qt::AlignHCenter | Qt::AlignVCenter);

  setLayout(panelLayout);
}

//-------------------------------------------t---------------------------------

void MeasurementControlTab::set_default_measurement_message()
{
  measurement_msg.MeasureLength=0;
  measurement_msg.MeasureDistanceToPlane=0;
  measurement_msg.EstimateLine=0;
  measurement_msg.EstimatePlane=0;
  measurement_msg.EstimateCircle=0;
  measurement_msg.EstimateRigidTransform=0;
  measurement_msg.Crop=0;
  measurement_msg.Undo=0;
  measurement_msg.CursorMaxDistance           = 0.05;
  measurement_msg.LineRoughMaxDistance        = 0.1;
  measurement_msg.LineMaxDistance             = 0.05;
  measurement_msg.PlanePrismDistance          = 0.1;
  measurement_msg.PlaneMaxDistance            = 0.025;
  measurement_msg.PlaneHullScaleFactor        = 2.0;
  measurement_msg.PlaneHullCrop               = 0;
  measurement_msg.PlaneFitTwice               = 1;
  measurement_msg.CircleMinRadius             = 0.15;
  measurement_msg.CircleMaxRadius             = 0.30;
  measurement_msg.CircleMaxDistance           = 0.02;
  measurement_msg.CirclePlanePrismDistance    = 0.025;
}

//----------------------------------------------------------------------------

MeasurementControlTab::~MeasurementControlTab()
{
  refreshManager->alive = false;
  refreshManager->quit();
  refreshManager->wait();
}

//----------------------------------------------------------------------------

MeasurementControlTab::MeasurementControlTab(QWidget *parent) : QTabWidget(parent)
{
  int argc = 0;
  char **argv = NULL;
  
  ros::init(argc, argv, "measurement panel");
  measurement_pub = n.advertise<ud_measurement_panel::MeasurementCommand>("measurement_command", 1000);

  set_default_measurement_message();
  
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
                 
  // Load Tabs
    
  initializeEstimationTab();
  std::cerr << "Estimation Tab Loaded" << std::endl;
  
  initializeRegistrationTab();
  std::cerr << "Registration Tab Loaded" << std::endl;

  initializeMetricsTab();
  std::cerr << "Metrics Tab Loaded" << std::endl;
  
  initializeSelectionTab();
  std::cerr << "Selection Tab Loaded" << std::endl;

  initializeDisplayTab();
  std::cerr << "Display Tab Loaded" << std::endl;
  
  addTab(estimationTab, "Estimate");
  addTab(registrationTab, "Register");
  addTab(metricsTab, "Metrics");
  addTab(selectionTab, "Select");
  addTab(displayTab, "Display");
  
  refreshManager = new RVizRefreshManager;
  refreshManager->parentWidget = this;
  connect(this, SIGNAL(sendWaitTime(int)), refreshManager, SLOT(getWaitTime(int)));
  refreshManager->start();
}

//----------------------------------------------------------------------------

void MeasurementControlTab::initializeRegistrationTab()
{
  QVBoxLayout *layout = new QVBoxLayout;

  // START ADDING TO LAYOUT
  
  // MOVE BOX 

  /*
  QGroupBox *moveBox = new QGroupBox;
  moveBox->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  moveBox->setAlignment(Qt::AlignHCenter | Qt::AlignTop);
  moveBox->setStyleSheet(groupStyleSheet);
  moveBox->setTitle("Move");
  
  // Move buttons layout
  QGridLayout *moveLayout = new QGridLayout;
  moveLayout->setAlignment(Qt::AlignHCenter | Qt::AlignTop);

  // Move Last Point Button
  QPushButton *btnMoveSelected = new QPushButton;
  btnMoveSelected->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  btnMoveSelected->setText("Selected");
  moveLayout->addWidget(btnMoveSelected, 0, 1, 1, 1, Qt::AlignHCenter | Qt::AlignVCenter);
  //  panelLayout->addWidget(btnMoveSelected, 0, 1, 1, 1, Qt::AlignHCenter | Qt::AlignVCenter);
  
  // Connect Button
  connect(btnMoveSelected, SIGNAL(clicked()), this, SLOT(btnMoveSelectedClick()));

  // Connect Box and Layout
  moveBox->setLayout(moveLayout);

  // DELETE BOX 

  QGroupBox *deleteBox = new QGroupBox;
  deleteBox->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  deleteBox->setAlignment(Qt::AlignHCenter | Qt::AlignTop);
  deleteBox->setStyleSheet(groupStyleSheet);
  deleteBox->setTitle("Delete");
  
  // Delete buttons layout
  QGridLayout *deleteLayout = new QGridLayout;
  deleteLayout->setAlignment(Qt::AlignHCenter | Qt::AlignTop);

  // Delete Last Point Button
  QPushButton *btnDeleteSelected = new QPushButton;
  btnDeleteSelected->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  btnDeleteSelected->setText("Selected");
  deleteLayout->addWidget(btnDeleteSelected, 0, 0, 1, 1, Qt::AlignHCenter | Qt::AlignVCenter);
  
  // Connect Button
  connect(btnDeleteSelected, SIGNAL(clicked()), this, SLOT(btnDeleteSelectedClick()));

  // Delete All Points Button
  QPushButton *btnDeleteAllPoints = new QPushButton;
  btnDeleteAllPoints->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  btnDeleteAllPoints->setText("All");
  deleteLayout->addWidget(btnDeleteAllPoints, 0, 1, 1, 1, Qt::AlignHCenter | Qt::AlignVCenter);
  
  // Connect Button
  connect(btnDeleteAllPoints, SIGNAL(clicked()), this, SLOT(btnDeleteAllClick()));
    
  
  // Connect Box and Layout
  deleteBox->setLayout(deleteLayout);
  */

  ///////////  End of Removing Points //////////////////

  // FINALIZE TAB
    
  layout->addWidget(InitializeEstimateRigidTransformBox(), Qt::AlignHCenter | Qt::AlignTop);

  //  layout->addWidget(moveBox, Qt::AlignHCenter | Qt::AlignTop);
  //  layout->addWidget(deleteBox, Qt::AlignHCenter | Qt::AlignTop);
  
  registrationTab = new QWidget;
  registrationTab->setLayout(layout);
  
}
  
//----------------------------------------------------------------------------

// text box for float64 CursorDistance outside of group boxes

// text box float64 RoughMaxLineDistance
// text box float64 MaxLineDistance

QGroupBox *MeasurementControlTab::InitializeEstimateLineBox()
{
  QGroupBox *estimationBox = new QGroupBox;
  estimationBox->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  estimationBox->setAlignment(Qt::AlignHCenter | Qt::AlignTop);
  estimationBox->setStyleSheet(groupStyleSheet);

  estimationBox->setTitle("Line");
  
  // Estimate buttons layout
  QGridLayout *estimateLayout = new QGridLayout;
  estimateLayout->setAlignment(Qt::AlignHCenter | Qt::AlignTop);

 // Rough max distance text box

  QLabel* labelLineRoughMaxDistance = new QLabel;
  labelLineRoughMaxDistance->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  labelLineRoughMaxDistance->setText("Rough max dist");
  //  labelLineRoughMaxDistance->setToolTip("Tilt angle to start sweep at (negative is down)");
  estimateLayout->addWidget(labelLineRoughMaxDistance, 0, 0, 1, 1, Qt::AlignLeft | Qt::AlignBottom );

  std::stringstream ss_rmd;
  ss_rmd << measurement_msg.LineRoughMaxDistance;

  textLineRoughMaxDistance = new QLineEdit;
  textLineRoughMaxDistance->setMaxLength(6);
  textLineRoughMaxDistance->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  textLineRoughMaxDistance->setReadOnly(false);
  textLineRoughMaxDistance->setText(ss_rmd.str().c_str());
  textLineRoughMaxDistance->setAlignment(Qt::AlignRight);
  estimateLayout->addWidget(textLineRoughMaxDistance, 1, 0, 1, 1, Qt::AlignCenter);

  // Max distance text box

  QLabel* labelLineMaxDistance = new QLabel;
  labelLineMaxDistance->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  labelLineMaxDistance->setText("Max distance");
  //  labelLineMaxDistance->setToolTip("Tilt angle to finish sweep at");
  estimateLayout->addWidget(labelLineMaxDistance, 0, 1, 1, 1, Qt::AlignLeft | Qt::AlignBottom );

  std::stringstream ss_md;
  ss_md << measurement_msg.LineMaxDistance;

  textLineMaxDistance = new QLineEdit;
  textLineMaxDistance->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  textLineMaxDistance->setMaxLength(6);
  textLineMaxDistance->setReadOnly(false);
  textLineMaxDistance->setText(ss_md.str().c_str());
  textLineMaxDistance->setAlignment(Qt::AlignRight);
  estimateLayout->addWidget(textLineMaxDistance, 1, 1, 1, 1, Qt::AlignCenter);

  
  // Estimate Line Button
  QPushButton *btnEstimateLine = new QPushButton;
  btnEstimateLine->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  btnEstimateLine->setText("OK");
  estimateLayout->addWidget(btnEstimateLine, 1, 2, 1, 1, Qt::AlignHCenter | Qt::AlignVCenter);
  
  // Connect Button
  connect(btnEstimateLine, SIGNAL(clicked()), this, SLOT(btnEstimateLineClick()));
    
  // Connect Box and Layout
  estimationBox->setLayout(estimateLayout);

  return estimationBox;
}

//----------------------------------------------------------------------------

// text box float64 PrismPlaneDistance > 0
// text box float64 MaxPlaneDistance
// text box float64 PlaneHullScaleFactor > 0
// check uint8 PlaneHullCrop
// check box uint8 PlaneFitTwice

// text box for prism_distance_threshold (aka rough max plane distance)
// check box for whether to do second fit (fit_twice)
// check box for whether to crop to hull
// text box for max_plane_distance (second fit)

QGroupBox *MeasurementControlTab::InitializeEstimatePlaneBox()
{
  QGroupBox *estimationBox = new QGroupBox;
  estimationBox->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  estimationBox->setAlignment(Qt::AlignHCenter | Qt::AlignTop);
  estimationBox->setStyleSheet(groupStyleSheet);

  estimationBox->setTitle("Plane");
  
  // Estimate buttons layout
  QGridLayout *estimateLayout = new QGridLayout;
  estimateLayout->setAlignment(Qt::AlignHCenter | Qt::AlignTop);
  
  // Prism distance text box

  QLabel* labelPlanePrismDistance = new QLabel;
  labelPlanePrismDistance->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  labelPlanePrismDistance->setText("Prism distance");
  //  labelPlanePrismDistance->setToolTip("Tilt angle to start sweep at (negative is down)");
  estimateLayout->addWidget(labelPlanePrismDistance, 0, 0, 1, 1, Qt::AlignLeft | Qt::AlignBottom );

  std::stringstream ss_pd;
  ss_pd << measurement_msg.PlanePrismDistance;

  textPlanePrismDistance = new QLineEdit;
  textPlanePrismDistance->setMaxLength(6);
  textPlanePrismDistance->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  textPlanePrismDistance->setReadOnly(false);
  textPlanePrismDistance->setText(ss_pd.str().c_str());
  textPlanePrismDistance->setAlignment(Qt::AlignRight);
  estimateLayout->addWidget(textPlanePrismDistance, 1, 0, 1, 1, Qt::AlignCenter);

  // Max distance text box

  QLabel* labelPlaneMaxDistance = new QLabel;
  labelPlaneMaxDistance->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  labelPlaneMaxDistance->setText("Max distance");
  //  labelPlaneMaxDistance->setToolTip("Tilt angle to finish sweep at");
  estimateLayout->addWidget(labelPlaneMaxDistance, 0, 1, 1, 1, Qt::AlignLeft | Qt::AlignBottom );

  std::stringstream ss_md;
  ss_md << measurement_msg.PlaneMaxDistance;

  textPlaneMaxDistance = new QLineEdit;
  textPlaneMaxDistance->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  textPlaneMaxDistance->setMaxLength(6);
  textPlaneMaxDistance->setReadOnly(false);
  textPlaneMaxDistance->setText(ss_md.str().c_str());
  textPlaneMaxDistance->setAlignment(Qt::AlignRight);
  estimateLayout->addWidget(textPlaneMaxDistance, 1, 1, 1, 1, Qt::AlignCenter);

  // Hull scale text box

  QLabel* labelPlaneHullScale = new QLabel;
  labelPlaneHullScale->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  labelPlaneHullScale->setText("Hull scale");
  //  labelPlaneHullScale->setToolTip("Tilt speed during sweep");
  estimateLayout->addWidget(labelPlaneHullScale, 0, 2, 1, 1, Qt::AlignLeft | Qt::AlignBottom );

  std::stringstream ss_hs;
  ss_hs << measurement_msg.PlaneHullScaleFactor;
  
  textPlaneHullScale = new QLineEdit;
  textPlaneHullScale->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  textPlaneHullScale->setMaxLength(5);
  textPlaneHullScale->setReadOnly(false);
  textPlaneHullScale->setText(ss_hs.str().c_str());
  textPlaneHullScale->setAlignment(Qt::AlignRight);
  estimateLayout->addWidget(textPlaneHullScale, 1, 2, 1, 1, Qt::AlignCenter);

  // Crop to hull checkbox

  checkPlaneHullCrop = new QCheckBox("Crop to hull", this);
  checkPlaneHullCrop->setChecked(measurement_msg.PlaneHullCrop);
  estimateLayout->addWidget(checkPlaneHullCrop, 2, 0, 1, 1, Qt::AlignCenter);

  // Fit twice checkbox

  checkPlaneFitTwice = new QCheckBox("Fit twice", this);
  checkPlaneFitTwice->setChecked(measurement_msg.PlaneFitTwice);
  estimateLayout->addWidget(checkPlaneFitTwice, 2, 1, 1, 1, Qt::AlignCenter);

  // Estimate Plane Button
  QPushButton *btnEstimatePlane = new QPushButton;
  btnEstimatePlane->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  btnEstimatePlane->setText("OK");
  estimateLayout->addWidget(btnEstimatePlane, 2, 2, 1, 1, Qt::AlignHCenter | Qt::AlignVCenter);
  
  // Connect Button
  connect(btnEstimatePlane, SIGNAL(clicked()), this, SLOT(btnEstimatePlaneClick()));

  // Connect Box and Layout
  estimationBox->setLayout(estimateLayout);

  return estimationBox;
}

//----------------------------------------------------------------------------

// text box float64 MinCircleRadius  > 0
// text box float64 MaxCircleRadius  > MinCircleRadius
// text box float64 MaxCircleDistance > 0
// text box float64 CirclePrismPlaneDistance > 0

// calls plane fit with prism_distance_threshold parameter different
// text box for min_radius
// text box for max_radius
// text box for inlier threshold distance for circle fit

QGroupBox *MeasurementControlTab::InitializeEstimateCircleBox()
{
  QGroupBox *estimationBox = new QGroupBox;
  estimationBox->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  estimationBox->setAlignment(Qt::AlignHCenter | Qt::AlignTop);
  estimationBox->setStyleSheet(groupStyleSheet);

  estimationBox->setTitle("Circle");
  
  // Estimate buttons layout
  QGridLayout *estimateLayout = new QGridLayout;
  estimateLayout->setAlignment(Qt::AlignHCenter | Qt::AlignTop);

  // Circle prism distance text box

  QLabel* labelCirclePlanePrismDistance = new QLabel;
  labelCirclePlanePrismDistance->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  labelCirclePlanePrismDistance->setText("Plane prism dist");
  //  labelCirclePlanePrismDistance->setToolTip("Tilt angle to start sweep at (negative is down)");
  estimateLayout->addWidget(labelCirclePlanePrismDistance, 0, 0, 1, 1, Qt::AlignLeft | Qt::AlignBottom );

  std::stringstream ss_pd;
  ss_pd << measurement_msg.CirclePlanePrismDistance;

  textCirclePlanePrismDistance = new QLineEdit;
  textCirclePlanePrismDistance->setMaxLength(6);
  textCirclePlanePrismDistance->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  textCirclePlanePrismDistance->setReadOnly(false);
  textCirclePlanePrismDistance->setText(ss_pd.str().c_str());
  textCirclePlanePrismDistance->setAlignment(Qt::AlignRight);
  estimateLayout->addWidget(textCirclePlanePrismDistance, 1, 0, 1, 1, Qt::AlignCenter);

  // Max distance text box

  QLabel* labelCircleMaxDistance = new QLabel;
  labelCircleMaxDistance->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  labelCircleMaxDistance->setText("Max circle dist");
  //  labelCircleMaxDistance->setToolTip("Tilt angle to finish sweep at");
  estimateLayout->addWidget(labelCircleMaxDistance, 0, 1, 1, 1, Qt::AlignLeft | Qt::AlignBottom );

  std::stringstream ss_md;
  ss_md << measurement_msg.CircleMaxDistance;

  textCircleMaxDistance = new QLineEdit;
  textCircleMaxDistance->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  textCircleMaxDistance->setMaxLength(6);
  textCircleMaxDistance->setReadOnly(false);
  textCircleMaxDistance->setText(ss_md.str().c_str());
  textCircleMaxDistance->setAlignment(Qt::AlignRight);
  estimateLayout->addWidget(textCircleMaxDistance, 1, 1, 1, 1, Qt::AlignCenter);

  // Min radius text box

  QLabel* labelCircleMinRadius = new QLabel;
  labelCircleMinRadius->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  labelCircleMinRadius->setText("Min radius");
  //  labelCircleMinRadius->setToolTip("Tilt angle to finish sweep at");
  estimateLayout->addWidget(labelCircleMinRadius, 2, 0, 1, 1, Qt::AlignLeft | Qt::AlignBottom );

  std::stringstream ss_minr;
  ss_minr << measurement_msg.CircleMinRadius;

  textCircleMinRadius = new QLineEdit;
  textCircleMinRadius->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  textCircleMinRadius->setMaxLength(6);
  textCircleMinRadius->setReadOnly(false);
  textCircleMinRadius->setText(ss_minr.str().c_str());
  textCircleMinRadius->setAlignment(Qt::AlignRight);
  estimateLayout->addWidget(textCircleMinRadius, 3, 0, 1, 1, Qt::AlignCenter);

  // Max radius text box

  QLabel* labelCircleMaxRadius = new QLabel;
  labelCircleMaxRadius->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  labelCircleMaxRadius->setText("Max radius");
  //  labelCircleMaxRadius->setToolTip("Tilt angle to finish sweep at");
  estimateLayout->addWidget(labelCircleMaxRadius, 2, 1, 1, 1, Qt::AlignLeft | Qt::AlignBottom );

  std::stringstream ss_maxr;
  ss_maxr << measurement_msg.CircleMaxRadius;

  textCircleMaxRadius = new QLineEdit;
  textCircleMaxRadius->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  textCircleMaxRadius->setMaxLength(6);
  textCircleMaxRadius->setReadOnly(false);
  textCircleMaxRadius->setText(ss_maxr.str().c_str());
  textCircleMaxRadius->setAlignment(Qt::AlignRight);
  estimateLayout->addWidget(textCircleMaxRadius, 3, 1, 1, 1, Qt::AlignCenter);
  
  // Estimate Circle Button
  QPushButton *btnEstimateCircle = new QPushButton;
  btnEstimateCircle->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  btnEstimateCircle->setText("OK");
  estimateLayout->addWidget(btnEstimateCircle, 3, 2, 1, 1, Qt::AlignHCenter | Qt::AlignVCenter);
  
  // Connect Button
  connect(btnEstimateCircle, SIGNAL(clicked()), this, SLOT(btnEstimateCircleClick()));

  // Connect Box and Layout
  estimationBox->setLayout(estimateLayout);

  return estimationBox;
}

//----------------------------------------------------------------------------

QGroupBox *MeasurementControlTab::InitializeEstimateRigidTransformBox()
{
  QGroupBox *estimationBox = new QGroupBox;
  estimationBox->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  estimationBox->setAlignment(Qt::AlignHCenter | Qt::AlignTop);
  estimationBox->setStyleSheet(groupStyleSheet);

  estimationBox->setTitle("Transform");
  
  // Estimate buttons layout
  QGridLayout *estimateLayout = new QGridLayout;
  estimateLayout->setAlignment(Qt::AlignHCenter | Qt::AlignTop);
  
  // Estimate Rigid Transform Button
  QPushButton *btnEstimateRigidTransform = new QPushButton;
  btnEstimateRigidTransform->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  btnEstimateRigidTransform->setText("OK");
  estimateLayout->addWidget(btnEstimateRigidTransform, 1, 1, 1, 1, Qt::AlignHCenter | Qt::AlignVCenter);
  
  // Connect Button
  connect(btnEstimateRigidTransform, SIGNAL(clicked()), this, SLOT(btnEstimateRigidTransformClick()));
  
  // Connect Box and Layout
  estimationBox->setLayout(estimateLayout);

  return estimationBox;
}
  
//----------------------------------------------------------------------------

void MeasurementControlTab::initializeEstimationTab()
{
  QVBoxLayout *layout = new QVBoxLayout;

  // START ADDING TO LAYOUT
  
  // Max cursor distance text box

  QLabel* labelCursorMaxDistance = new QLabel;
  labelCursorMaxDistance->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  labelCursorMaxDistance->setText("Max cursor dist");
  //  labelCursorMaxDistance->setToolTip("Tilt angle to finish sweep at");
  layout->addWidget(labelCursorMaxDistance, Qt::AlignLeft | Qt::AlignBottom );

  std::stringstream ss_md;
  ss_md << measurement_msg.CursorMaxDistance;

  textCursorMaxDistance = new QLineEdit;
  textCursorMaxDistance->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  textCursorMaxDistance->setMaxLength(6);
  textCursorMaxDistance->setReadOnly(false);
  textCursorMaxDistance->setText(ss_md.str().c_str());
  textCursorMaxDistance->setAlignment(Qt::AlignRight);
  layout->addWidget(textCursorMaxDistance, Qt::AlignLeft | Qt::AlignBottom);

  // FINALIZE TAB
    
  layout->addWidget(InitializeEstimateLineBox(), Qt::AlignHCenter | Qt::AlignTop);
  layout->addWidget(InitializeEstimatePlaneBox(), Qt::AlignHCenter | Qt::AlignTop);
  layout->addWidget(InitializeEstimateCircleBox(), Qt::AlignHCenter | Qt::AlignTop);
  
  estimationTab = new QWidget;
  estimationTab->setLayout(layout);
    
}

//----------------------------------------------------------------------------

void MeasurementControlTab::initializeMetricsTab()
{
  QVBoxLayout *layout = new QVBoxLayout;

  // START ADDING TO LAYOUT

  //Create a group box for "Measure" action

  QGroupBox *measureBox = new QGroupBox;
  measureBox->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  measureBox->setAlignment(Qt::AlignHCenter | Qt::AlignTop);
  measureBox->setStyleSheet(groupStyleSheet);
  //  measureBox->setTitle("Measure");
  
  // Measure buttons layout
  QGridLayout *measureLayout = new QGridLayout;
  measureLayout->setAlignment(Qt::AlignHCenter | Qt::AlignTop);
  
  // Measure Line Button
  QPushButton *btnMeasureLine = new QPushButton;
  btnMeasureLine->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  btnMeasureLine->setText("Length");
  measureLayout->addWidget(btnMeasureLine, 0, 0, 1, 1, Qt::AlignHCenter | Qt::AlignVCenter);
  
  // Connect Button
  connect(btnMeasureLine, SIGNAL(clicked()), this, SLOT(btnMeasureLengthClick()));
  
  // Connect Box and Layout
  measureBox->setLayout(measureLayout);
  
  ////////// End of Measure ///////////////
  

  // FINALIZE TAB

  layout->addWidget(measureBox, Qt::AlignHCenter | Qt::AlignTop);

  metricsTab = new QWidget;
  metricsTab->setLayout(layout);
}

//----------------------------------------------------------------------------

void MeasurementControlTab::initializeSelectionTab()
{
  QVBoxLayout *layout = new QVBoxLayout;

  // START ADDING TO LAYOUT

  //Create a group box for "Crop" action

  QGroupBox *cropBox = new QGroupBox;
  cropBox->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  cropBox->setAlignment(Qt::AlignHCenter | Qt::AlignTop);
  cropBox->setStyleSheet(groupStyleSheet);
  cropBox->setTitle("Crop");
  
  // Estimate buttons layout
  QGridLayout *cropLayout = new QGridLayout;
  cropLayout->setAlignment(Qt::AlignHCenter | Qt::AlignTop);
  
  // Crop Button
  QPushButton *btnCrop = new QPushButton;
  btnCrop->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  btnCrop->setText("Crop");
  cropLayout->addWidget(btnCrop, 0, 0, 1, 1, Qt::AlignHCenter | Qt::AlignVCenter);
  
  // Connect Button
  connect(btnCrop, SIGNAL(clicked()), this, SLOT(btnCropClick()));
    
  // Undo Button
  QPushButton *btnUndo = new QPushButton;
  btnUndo->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  btnUndo->setText("Undo");
  cropLayout->addWidget(btnUndo, 0, 1, 1, 1, Qt::AlignHCenter | Qt::AlignVCenter);
  
   // Connect Button
  connect(btnUndo, SIGNAL(clicked()), this, SLOT(btnUndoClick()));
    
  // Connect Box and Layout
  cropBox->setLayout(cropLayout);
  
  ////////// End of Crop ///////////////

  // FINALIZE TAB

  layout->addWidget(cropBox, Qt::AlignHCenter | Qt::AlignTop);

  selectionTab = new QWidget;
  selectionTab->setLayout(layout);
}

//----------------------------------------------------------------------------

void MeasurementControlTab::initializeDisplayTab()
{
  QVBoxLayout *layout = new QVBoxLayout;

  // START ADDING TO LAYOUT


  // FINALIZE TAB

  displayTab = new QWidget;
  displayTab->setLayout(layout);
}

//----------------------------------------------------------------------------

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

//----------------------------------------------------------------------------

void RVizRefreshManager::getWaitTime(int t)
{
    waitTime = t;
}

//----------------------------------------------------------------------------

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

//----------------------------------------------------------------------------

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

//----------------------------------------------------------------------------

} // ud_measurement_space namespace

//----------------------------------------------------------------------------

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( ud_measurement_space::MeasurementPanel,rviz::Panel )
PLUGINLIB_EXPORT_CLASS( ud_measurement_space::MeasurementControlTab, QTabWidget )
PLUGINLIB_EXPORT_CLASS( ud_measurement_space::RVizRefreshManager, QThread )

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
