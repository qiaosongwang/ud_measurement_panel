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

void MeasurementControlTab::btnRemoveAllPointsClick()
{
    std::cerr << "Remove All Points Clicked"<< std::endl;
    ros::Rate loop_rate(10);
    loop_rate.sleep();
    ud_measurement_panel::MeasurementCommand msg;
    msg.RemoveAllPoints = 1;
    measurement_pub.publish(msg);
    ros::spinOnce();
}

void MeasurementControlTab::btnRemoveLastPointClick()
{
    std::cerr << "Remove Last Point Clicked"<< std::endl;
    ros::Rate loop_rate(10);
    loop_rate.sleep();
    ud_measurement_panel::MeasurementCommand msg;
    msg.RemoveLastPoint = 1;
    measurement_pub.publish(msg);
    ros::spinOnce();
}

void MeasurementControlTab::btnEstimateLineClick()
{
    std::cerr << "Est Line Clicked"<< std::endl;
    ros::Rate loop_rate(10);
    loop_rate.sleep();
    ud_measurement_panel::MeasurementCommand msg;
    msg.EstimateLine = 1;
    measurement_pub.publish(msg);
    ros::spinOnce();
}
void MeasurementControlTab::btnEstimatePlaneClick()
{
    std::cerr << "Est Plane Clicked"<< std::endl;
    ros::Rate loop_rate(10);
    loop_rate.sleep();
    ud_measurement_panel::MeasurementCommand msg;
    msg.EstimatePlane = 1;
    measurement_pub.publish(msg);
    ros::spinOnce();
}
void MeasurementControlTab::btnMeasureLengthClick()
{
    std::cerr << "Measure Length Clicked"<< std::endl;
    ros::Rate loop_rate(10);
    loop_rate.sleep();
    ud_measurement_panel::MeasurementCommand msg;
    msg.MeasureLength = 1;
    measurement_pub.publish(msg);
    ros::spinOnce();
}
void MeasurementControlTab::btnCropClick()
{
    std::cerr << "Crop Clicked"<< std::endl;
    ros::Rate loop_rate(10);
    loop_rate.sleep();
    ud_measurement_panel::MeasurementCommand msg;
    msg.Crop = 1;
    measurement_pub.publish(msg);
    ros::spinOnce();
}
void MeasurementControlTab::btnUndoClick()
{
    std::cerr << "Undo Clicked"<< std::endl;
    ros::Rate loop_rate(10);
    loop_rate.sleep();
    ud_measurement_panel::MeasurementCommand msg;
    msg.Undo = 1;
    measurement_pub.publish(msg);
    ros::spinOnce();
}

} // End: namespace ud_measurement_space
