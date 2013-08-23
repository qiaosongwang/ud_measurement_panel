/*
 * Brad Fletcher
 * brad@udel.edu
 * Thanks to Georgia Tech for allowing me to use their 
 * RViz panel code as a model for this.
 */

#ifndef MEASUREMENT_CONTROL_H
#define MEASUREMENT_CONTROL_H

#include <stdio.h>

#include <QApplication>
#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <ros/ros.h>
#include <QTableWidget>
#include <QPushButton>
#include <QLineEdit>
#include <QRadioButton>
#include <QSpinBox>
#include <QLabel>
#include <QProcess>
#include <QGroupBox>
#include <QButtonGroup>
#include <QProcess>
#include <QString>
#include <QStringList>
#include <QTextStream>
#include <QClipboard>
#include <QPalette>
#include <QColor>
#include <QThread>
#include <QComboBox>
#include <QCheckBox>

#include <vector>

#include <rviz/panel.h>

#include "ud_measurement_panel/MeasurementCommand.h"

namespace ud_measurement_space
{

  class MeasurementControlTab;

  class RVizRefreshManager : public QThread
  {
    Q_OBJECT
      public:
    MeasurementControlTab* parentWidget;
    bool alive;
    int waitTime;
    
  protected:
    virtual void run();
    
    protected slots:
    void getWaitTime(int t);
    
  signals:
    void signalRefresh();
    
  };
  
  // Here we declare our new subclass of rviz::Panel.  Every panel which
  // can be added via the Panels/Add_New_Panel menu is a subclass of
  // rviz::Panel.
  class MeasurementControlTab: public QTabWidget
  {
    // This class uses Qt slots and is a subclass of QObject, so it needs
    // the Q_OBJECT macro.
    Q_OBJECT
      public:
    // QWidget subclass constructors usually take a parent widget
    // parameter (which usually defaults to 0).  At the same time,
    // pluginlib::ClassLoader creates instances by calling the default
    // constructor (with no arguments).  Taking the parameter and giving
    // a default of 0 lets the default constructor work and also lets
    // someone using the class for something else to pass in a parent
    // widget as they normally would with Qt.
    MeasurementControlTab( QWidget* parent = 0 );

    ~MeasurementControlTab();
        
    // ROS Related
      
    ros::NodeHandle n;
    ros::Publisher measurement_pub;
      
    QString groupStyleSheet;

    ud_measurement_panel::MeasurementCommand measurement_msg;

    // Update timer
    RVizRefreshManager* refreshManager;
    int getRefreshTime();
    
    // Slots will be "connected" to signals in order to respond to user events
  protected:
    //    int ipAddrA;
    //    int ipAddrB;
    //    int ipAddrC;
    //    int ipAddrD;
    
  signals:
    void sendWaitTime(int t);
    
    protected Q_SLOTS:      
    
    //Handle Button Clicks

    /*    
    void btnDeleteAllClick();
    void btnDeleteSelectedClick(); 
    void btnMoveSelectedClick(); 
    */

    void btnEstimateLineClick();
    void btnEstimatePlaneClick();
    void btnEstimateCircleClick();
    void btnEstimateRigidTransformClick();
    void btnMeasureLengthClick();
    void btnCropClick();
    void btnUndoClick();
    
    // Update all state information
    void refreshState();
    
  private:

    QLineEdit* textCursorMaxDistance;

    QLineEdit* textLineRoughMaxDistance;
    QLineEdit* textLineMaxDistance;
 
    QLineEdit* textPlanePrismDistance;
    QLineEdit* textPlaneMaxDistance;
    QLineEdit* textPlaneHullScale;
    QCheckBox *checkPlaneHullCrop;
    QCheckBox *checkPlaneFitTwice;

    QLineEdit* textCirclePlanePrismDistance;
    QLineEdit* textCircleMaxDistance;
    QLineEdit* textCircleMinRadius;
    QLineEdit* textCircleMaxRadius;

    void initializeRegistrationTab();
    QWidget* registrationTab;

    void initializeEstimationTab();
    QWidget* estimationTab;

    void initializeMetricsTab();
    QWidget* metricsTab;

    void initializeSelectionTab();
    QWidget* selectionTab;

    void initializeDisplayTab();
    QWidget* displayTab;
  
    QGroupBox *InitializeEstimateLineBox();
    QGroupBox *InitializeEstimatePlaneBox();
    QGroupBox *InitializeEstimateCircleBox();
    QGroupBox *InitializeEstimateRigidTransformBox();

    void set_default_measurement_message();
    void construct_current_measurement_message();

  };
  
  
  class MeasurementPanel : public rviz::Panel
  {
    Q_OBJECT
      public:
    MeasurementPanel(QWidget *parent = 0);
    
    // Now we declare overrides of rviz::Panel functions for saving and
    // loading data from the config file.  Here the data is the
    // topic name.
    virtual void load( const rviz::Config& config );
    virtual void save( rviz::Config config ) const;
    
    //  private:
    
    MeasurementControlTab* controlTabWidget;
    
  };
  
} // end namespace ud_measurement_space


#endif // MEASUREMENT_CONTROL_H
