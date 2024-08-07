// ------------------------------- ROS2 DEPENDENCIES --------------------------
#include <ros/ros.h>   // ROS Header
#include <controller_manager/controller_manager.h>

// ------------------------------ ADDITIONAL DEPDENDENCIES -------------------
#include "diffdrive_arduino/diffdrive_arduino.h"

// --------------------------- MAIN IMPLEMENTATION ---------------------------
int main(int argc, char **argv)
{
    // Initialize node
    ros::init(argc, argv, "diffdrive_robot");

    // Create node handler
    ros::NodeHandle n("~");

    // Declare robot controler configuration 
    DiffDriveArduino::Config robot_cfg;

    // Attempt to retrieve parameters. If they don't exist, the default values 
    // from the struct will be used
    n.getParam("left_wheel_name", robot_cfg.left_wheel_name);
    n.getParam("right_wheel_name", robot_cfg.right_wheel_name);
    n.getParam("baud_rate", robot_cfg.baud_rate);
    n.getParam("device", robot_cfg.device);
    n.getParam("enc_counts_per_rev", robot_cfg.enc_counts_per_rev);
    n.getParam("robot_loop_rate", robot_cfg.loop_rate);
    
    // Initialize robot controller and controller manager
    DiffDriveArduino robot(robot_cfg);
    controller_manager::ControllerManager cm(&robot);

    // Initialize asynchronous spinner
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Start timer count and set rates
    ros::Time prevTime = ros::Time::now();
    ros::Rate loop_rate(10);

    // While ROS is working
    while (ros::ok())
    {
        // Read encoders and update values
        robot.read();

        // Update timers
        cm.update(robot.get_time(), robot.get_period());
        
        // Write command velocities
        robot.write();
        loop_rate.sleep();
    }
}
