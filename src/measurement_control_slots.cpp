/*
 * Brad Fletcher
 * brad@udel.edu
 * Thanks to Georgia Tech for allowing me to use their 
 * RViz panel code as a model for this.
 */


#include "measurement_control.h"

//added
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ud_measurement_panel/MeasurementCommand.h"

namespace ud_measurement_space
{

void MeasurementControlTab::refreshState()
{
	
} 

void MeasurementControlTab::btnDeleteAllClick()
{
    std::cerr << "Delete All Points" << std::endl;
    ros::Rate loop_rate(10);
    loop_rate.sleep();
    ud_measurement_panel::MeasurementCommand msg;
    msg.DeleteAll = 1;
    measurement_pub.publish(msg);
    ros::spinOnce();
    msg.DeleteAll = 9;
}

void MeasurementControlTab::btnDeleteSelectedClick()
{
    std::cerr << "Delete Selected Point" << std::endl;
    ros::Rate loop_rate(10);
    loop_rate.sleep();
    ud_measurement_panel::MeasurementCommand msg;
    msg.DeleteSelected = 1;
    measurement_pub.publish(msg);
    ros::spinOnce();
    msg.DeleteSelected = 0;
}

void MeasurementControlTab::btnMoveSelectedClick()
{
    std::cerr << "Move Selected Point" << std::endl;
    ros::Rate loop_rate(10);
    loop_rate.sleep();
    ud_measurement_panel::MeasurementCommand msg;
    msg.MoveSelected = 1;
    measurement_pub.publish(msg);
    ros::spinOnce();
    msg.MoveSelected = 0;
}

void MeasurementControlTab::btnEstimateLineClick()
{
    std::cerr << "Estimate Line" << std::endl;
    ros::Rate loop_rate(10);
    loop_rate.sleep();
    ud_measurement_panel::MeasurementCommand msg;
    msg.EstimateLine = 1;
    measurement_pub.publish(msg);
    ros::spinOnce();
    msg.EstimateLine = 0;
}

void MeasurementControlTab::btnEstimatePlaneClick()
{
    std::cerr << "Estimate Plane" << std::endl;
    ros::Rate loop_rate(10);
    loop_rate.sleep();
    ud_measurement_panel::MeasurementCommand msg;
    msg.EstimatePlane = 1;
    measurement_pub.publish(msg);
    ros::spinOnce();
    msg.EstimatePlane = 0;
}

void MeasurementControlTab::btnEstimateCircleClick()
{
    std::cerr << "Estimate Circle" << std::endl;
    ros::Rate loop_rate(10);
    loop_rate.sleep();
    ud_measurement_panel::MeasurementCommand msg;
    msg.EstimateCircle = 1;
    measurement_pub.publish(msg);
    ros::spinOnce();
    msg.EstimateCircle = 0;
}

void MeasurementControlTab::btnEstimateRigidTransformClick()
{
    std::cerr << "Estimate RigidTransform" << std::endl;
    ros::Rate loop_rate(10);
    loop_rate.sleep();
    ud_measurement_panel::MeasurementCommand msg;
    msg.EstimateRigidTransform = 1;
    measurement_pub.publish(msg);
    ros::spinOnce();
    msg.EstimateRigidTransform = 0;
}

void MeasurementControlTab::btnMeasureLengthClick()
{
    std::cerr << "Measure Length" << std::endl;
    ros::Rate loop_rate(10);
    loop_rate.sleep();
    ud_measurement_panel::MeasurementCommand msg;
    msg.MeasureLength = 1;
    measurement_pub.publish(msg);
    ros::spinOnce();
    msg.MeasureLength = 0;
}

void MeasurementControlTab::btnCropClick()
{
    std::cerr << "Crop" << std::endl;
    ros::Rate loop_rate(10);
    loop_rate.sleep();
    ud_measurement_panel::MeasurementCommand msg;
    msg.Crop = 1;
    measurement_pub.publish(msg);
    ros::spinOnce();
    msg.Crop = 0;
}

void MeasurementControlTab::btnUndoClick()
{
    std::cerr << "Undo" << std::endl;
    ros::Rate loop_rate(10);
    loop_rate.sleep();
    ud_measurement_panel::MeasurementCommand msg;
    msg.Undo = 1;
    measurement_pub.publish(msg);
    ros::spinOnce();
    msg.Undo = 0;
}

} // End: namespace ud_measurement_space
