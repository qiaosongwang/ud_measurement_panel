/*
 * Brad Fletcher
 * brad@udel.edu
 * Thanks to Georgia Tech for allowing me to use their 
 * RViz panel code as a model for this.
 */


#include "sensor_control.h"

//added
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "rviz_sensor_control_panel/HokuyoCommand.h"
#include "rviz_sensor_control_panel/FleaCommand.h"
#include "ud_measurement_panel/MeasurementCommand.h"

namespace ud_measurement_space
{

void MeasurementControlTab::refreshState()
{
	
} 

void MeasurementControlTab::hokuyoEditHandle()
{
    std::cerr << "Hokuyo Scan: start at: " << txtMinTheta->text().toStdString() << std::endl;
    
    //added ros stuff
 
    ros::Rate loop_rate(10);
    loop_rate.sleep();

    std::cerr << "hello!\n";

    printf("hokuyo edit\n"); fflush(stdout);

    
    rviz_sensor_control_panel::HokuyoCommand msg;
    msg.minTheta = txtMinTheta->text().toFloat();
    //msg.maxTheta = txtMaxTheta->text().toFloat();
    //msg.degreesPerSecond = txtDPS->text().toFloat();
    hokuyo_pub.publish(msg);
    

    //ROS_INFO("%s", msg.data.c_str());
    ros::spinOnce();

    //    loop_rate.sleep();
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
