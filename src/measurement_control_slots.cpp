/*
 * Brad Fletcher
 * brad@udel.edu
 * Thanks to Georgia Tech for allowing me to use their 
 * RViz panel code as a model for this.
 */


#include "measurement_control.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ud_measurement_panel/MeasurementCommand.h"

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------

namespace ud_measurement_space
{

//----------------------------------------------------------------------------

void MeasurementControlTab::refreshState()
{
	
} 

//----------------------------------------------------------------------------

void MeasurementControlTab::btnDeleteAllClick()
{
    std::cerr << "Delete All Points" << std::endl;
    ros::Rate loop_rate(10);
    loop_rate.sleep();
    //    ud_measurement_panel::MeasurementCommand msg;
    measurement_msg.DeleteAll = 1;
    measurement_pub.publish(measurement_msg);
    ros::spinOnce();
    measurement_msg.DeleteAll = 0;
}

//----------------------------------------------------------------------------

void MeasurementControlTab::btnDeleteSelectedClick()
{
    std::cerr << "Delete Selected Point" << std::endl;
    ros::Rate loop_rate(10);
    loop_rate.sleep();
    //    ud_measurement_panel::MeasurementCommand msg;
    measurement_msg.DeleteSelected = 1;
    measurement_pub.publish(measurement_msg);
    ros::spinOnce();
    measurement_msg.DeleteSelected = 0;
}

//----------------------------------------------------------------------------

void MeasurementControlTab::btnMoveSelectedClick()
{
    std::cerr << "Move Selected Point" << std::endl;
    ros::Rate loop_rate(10);
    loop_rate.sleep();
    //    ud_measurement_panel::MeasurementCommand msg;
    measurement_msg.MoveSelected = 1;
    measurement_pub.publish(measurement_msg);
    ros::spinOnce();
    measurement_msg.MoveSelected = 0;
}

//----------------------------------------------------------------------------

void MeasurementControlTab::construct_current_measurement_message()
{
  measurement_msg.CursorMaxDistance = textCursorMaxDistance->text().toFloat();
  
  measurement_msg.LineRoughMaxDistance = textLineRoughMaxDistance->text().toFloat();
  measurement_msg.LineMaxDistance = textLineMaxDistance->text().toFloat();
  
  measurement_msg.PlanePrismDistance = textPlanePrismDistance->text().toFloat();
  measurement_msg.PlaneMaxDistance = textPlaneMaxDistance->text().toFloat();
  measurement_msg.PlaneHullScaleFactor = textPlaneHullScale->text().toFloat();
  measurement_msg.PlaneHullCrop = checkPlaneHullCrop->isChecked();
  measurement_msg.PlaneFitTwice = checkPlaneFitTwice->isChecked();
  
  measurement_msg.CirclePlanePrismDistance = textCirclePlanePrismDistance->text().toFloat();
  measurement_msg.CircleMaxDistance = textCircleMaxDistance->text().toFloat();
  measurement_msg.CircleMinRadius = textCircleMinRadius->text().toFloat();
  measurement_msg.CircleMaxRadius = textCircleMaxRadius->text().toFloat();
}

//----------------------------------------------------------------------------

void MeasurementControlTab::btnEstimateLineClick()
{
    std::cerr << "Estimate Line" << std::endl;
    ros::Rate loop_rate(10);
    loop_rate.sleep();
    //    ud_measurement_panel::MeasurementCommand msg;

    construct_current_measurement_message();

    measurement_msg.EstimateLine = 1;
    measurement_pub.publish(measurement_msg);
    ros::spinOnce();
    measurement_msg.EstimateLine = 0;
}

//----------------------------------------------------------------------------

void MeasurementControlTab::btnEstimatePlaneClick()
{
    std::cerr << "Estimate Plane" << std::endl;
    ros::Rate loop_rate(10);
    loop_rate.sleep();

    construct_current_measurement_message();

    printf("%lf %lf %lf\n", measurement_msg.PlanePrismDistance, measurement_msg.PlaneMaxDistance, measurement_msg.PlaneHullScaleFactor);

    measurement_msg.EstimatePlane = 1;
    measurement_pub.publish(measurement_msg);
    ros::spinOnce();
    measurement_msg.EstimatePlane = 0;
}

//----------------------------------------------------------------------------

void MeasurementControlTab::btnEstimateCircleClick()
{
    std::cerr << "Estimate Circle" << std::endl;
    ros::Rate loop_rate(10);
    loop_rate.sleep();
    //    ud_measurement_panel::MeasurementCommand msg;

    construct_current_measurement_message();

    measurement_msg.EstimateCircle = 1;
    measurement_pub.publish(measurement_msg);
    ros::spinOnce();
    measurement_msg.EstimateCircle = 0;
}

//----------------------------------------------------------------------------

void MeasurementControlTab::btnEstimateRigidTransformClick()
{
    std::cerr << "Estimate RigidTransform" << std::endl;
    ros::Rate loop_rate(10);
    loop_rate.sleep();
    //    ud_measurement_panel::MeasurementCommand msg;
    measurement_msg.EstimateRigidTransform = 1;
    measurement_pub.publish(measurement_msg);
    ros::spinOnce();
    measurement_msg.EstimateRigidTransform = 0;
}

//----------------------------------------------------------------------------

void MeasurementControlTab::btnMeasureLengthClick()
{
    std::cerr << "Measure Length" << std::endl;
    ros::Rate loop_rate(10);
    loop_rate.sleep();
    //    ud_measurement_panel::MeasurementCommand msg;
    measurement_msg.MeasureLength = 1;
    measurement_pub.publish(measurement_msg);
    ros::spinOnce();
    measurement_msg.MeasureLength = 0;
}

//----------------------------------------------------------------------------

void MeasurementControlTab::btnCropClick()
{
    std::cerr << "Crop" << std::endl;
    ros::Rate loop_rate(10);
    loop_rate.sleep();
    //    ud_measurement_panel::MeasurementCommand msg;
    measurement_msg.Crop = 1;
    measurement_pub.publish(measurement_msg);
    ros::spinOnce();
    measurement_msg.Crop = 0;
}

//----------------------------------------------------------------------------

void MeasurementControlTab::btnUndoClick()
{
    std::cerr << "Undo" << std::endl;
    ros::Rate loop_rate(10);
    loop_rate.sleep();
    //    ud_measurement_panel::MeasurementCommand msg;
    measurement_msg.Undo = 1;
    measurement_pub.publish(measurement_msg);
    ros::spinOnce();
    measurement_msg.Undo = 0;
}

//----------------------------------------------------------------------------

} // End: namespace ud_measurement_space

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
