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

// this just collects parameter values from the various text boxes

void MeasurementControlTab::gather_current_measurement_command_parameters()
{
  measurement_command_msg.CursorMaxDistance = textCursorMaxDistance->text().toFloat();
  
  measurement_command_msg.LineRoughMaxDistance = textLineRoughMaxDistance->text().toFloat();
  measurement_command_msg.LineMaxDistance = textLineMaxDistance->text().toFloat();
  
  measurement_command_msg.PlanePrismDistance = textPlanePrismDistance->text().toFloat();
  measurement_command_msg.PlaneMaxDistance = textPlaneMaxDistance->text().toFloat();
  measurement_command_msg.PlaneHullScaleFactor = textPlaneHullScale->text().toFloat();
  measurement_command_msg.PlaneHullCrop = checkPlaneHullCrop->isChecked();
  measurement_command_msg.PlaneFitTwice = checkPlaneFitTwice->isChecked();
  
  measurement_command_msg.CirclePlanePrismDistance = textCirclePlanePrismDistance->text().toFloat();
  measurement_command_msg.CircleMaxDistance = textCircleMaxDistance->text().toFloat();
  measurement_command_msg.CircleMinRadius = textCircleMinRadius->text().toFloat();
  measurement_command_msg.CircleMaxRadius = textCircleMaxRadius->text().toFloat();
}

//----------------------------------------------------------------------------

void MeasurementControlTab::btnEstimateLineClick()
{
    std::cerr << "Estimate Line" << std::endl;
    ros::Rate loop_rate(10);
    loop_rate.sleep();

    set_default_measurement_command_type();
    gather_current_measurement_command_parameters();

    measurement_command_msg.EstimateLine = 1;
    measurement_pub.publish(measurement_command_msg);
    ros::spinOnce();

    //    measurement_command_msg.EstimateLine = 0;
}

//----------------------------------------------------------------------------

void MeasurementControlTab::btnEstimatePlaneClick()
{
    std::cerr << "Estimate Plane" << std::endl;
    ros::Rate loop_rate(10);
    loop_rate.sleep();

    set_default_measurement_command_type();
    gather_current_measurement_command_parameters();

    //    printf("%lf %lf %lf\n", measurement_command_msg.PlanePrismDistance, measurement_command_msg.PlaneMaxDistance, measurement_command_msg.PlaneHullScaleFactor);

    measurement_command_msg.EstimatePlane = 1;
    measurement_pub.publish(measurement_command_msg);
    ros::spinOnce();

    //    measurement_command_msg.EstimatePlane = 0;
}

//----------------------------------------------------------------------------

void MeasurementControlTab::btnEstimateCircleClick()
{
    std::cerr << "Estimate Circle" << std::endl;
    ros::Rate loop_rate(10);
    loop_rate.sleep();

    set_default_measurement_command_type();
    gather_current_measurement_command_parameters();

    measurement_command_msg.EstimateCircle = 1;
    measurement_pub.publish(measurement_command_msg);
    ros::spinOnce();

    //    measurement_command_msg.EstimateCircle = 0;
}

//----------------------------------------------------------------------------

void MeasurementControlTab::btnEstimateRigidTransformClick()
{
    std::cerr << "Estimate RigidTransform" << std::endl;
    ros::Rate loop_rate(10);
    loop_rate.sleep();

    set_default_measurement_command_type();
    gather_current_measurement_command_parameters();

    measurement_command_msg.EstimateRigidTransform = 1;
    measurement_pub.publish(measurement_command_msg);
    ros::spinOnce();

    //    measurement_command_msg.EstimateRigidTransform = 0;
}

//----------------------------------------------------------------------------

void MeasurementControlTab::btnMeasureLengthClick()
{
    std::cerr << "Measure Length" << std::endl;
    ros::Rate loop_rate(10);
    loop_rate.sleep();

    set_default_measurement_command_type();
    gather_current_measurement_command_parameters();

    measurement_command_msg.MeasureLength = 1;
    measurement_pub.publish(measurement_command_msg);
    ros::spinOnce();
 
   //    measurement_command_msg.MeasureLength = 0;
}

//----------------------------------------------------------------------------

void MeasurementControlTab::btnMeasureDistanceToPlaneClick()
{
    std::cerr << "Measure DistanceToPlane" << std::endl;
    ros::Rate loop_rate(10);
    loop_rate.sleep();

    set_default_measurement_command_type();
    gather_current_measurement_command_parameters();

    measurement_command_msg.MeasureDistanceToPlane = 1;
    measurement_pub.publish(measurement_command_msg);
    ros::spinOnce();

    //    measurement_command_msg.MeasureDistanceToPlane = 0;
}

//----------------------------------------------------------------------------

void MeasurementControlTab::btnCropClick()
{
    std::cerr << "Crop" << std::endl;
    ros::Rate loop_rate(10);
    loop_rate.sleep();

    set_default_measurement_command_type();
    gather_current_measurement_command_parameters();

    measurement_command_msg.Crop = 1;
    measurement_pub.publish(measurement_command_msg);
    ros::spinOnce();
 
   //   measurement_command_msg.Crop = 0;
}

//----------------------------------------------------------------------------

void MeasurementControlTab::btnUndoClick()
{
    std::cerr << "Undo" << std::endl;
    ros::Rate loop_rate(10);
    loop_rate.sleep();

    set_default_measurement_command_type();
    gather_current_measurement_command_parameters();

    measurement_command_msg.Undo = 1;
    measurement_pub.publish(measurement_command_msg);
    ros::spinOnce();

    //    measurement_command_msg.Undo = 0;
}

//----------------------------------------------------------------------------

} // End: namespace ud_measurement_space

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
