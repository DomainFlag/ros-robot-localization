#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"

// ROS::Publisher motor commands;
ros::Publisher motor_command_publisher;

// This function publishes the requested linear x and angular velocities to the robot wheel joints
// After publishing the requested velocities, a message feedback is returned with the requested wheel velocities
bool handle_drive_request(ball_chaser::DriveToTarget::Request & req, ball_chaser::DriveToTarget::Response & res) {
    ROS_INFO("DriveToTargetRequest received - linear_x:%1.2f, angular_z:%1.2f", req.linear_x, req.angular_z);
    
    // Velocity in free space broken into its linear and angular parts
    geometry_msgs::Twist message;

    message.linear.x = req.linear_x;
    message.angular.z = req.angular_z;

	// Publish message
    motor_command_publisher.publish(message);

    // Return a response message
    res.msg_feedback = "Motor command set - linear_x: " + std::to_string(req.linear_x) + " , angular_z: " + std::to_string(req.angular_z);
    ROS_INFO_STREAM(res.msg_feedback);

    return true;
}

int main(int argc, char** argv) {
    // Initialize a ROS node
    ros::init(argc, argv, "drive_bot");

    // Create a ROS NodeHandle object
    ros::NodeHandle n;

    // Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic with a publishing queue size of 10
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // Drive /ball_chaser/command_robot service
    ros::ServiceServer service = n.advertiseService("/ball_chaser/command_robot", handle_drive_request);
	ROS_INFO("Ready to listen for drive commands");

    // Handle ROS communication events
    ros::spin();

    return 0;
}
