#include <cmath>
#include <iostream>
#include <algorithm>
#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

using namespace std;

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z) {
    // Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget command;
    command.request.linear_x = lin_x;
    command.request.angular_z = ang_z;

    // Service command_robot call
    if(!client.call(command))
    	ROS_ERROR("Error service command_robot");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img) {

    int white_pixel = 255;

    // Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
	int counter[3] = {0};
	int stepSize = floor(img.step / img.width);
	int threshold = floor(img.width / 3);
	bool flag;
	for(int g = 0; g < img.height * img.step; g += stepSize) {
		flag = true;
		for(int h = 0; h < stepSize; h++) {
			if(img.data[g + h] != white_pixel) {
				flag = false;
				break;
			}
		}
		
		if(flag) {
			int index = floor((g % img.step) / stepSize / threshold);
			
			counter[index]++;
		}
	}
	
	int total = accumulate(begin(counter), end(counter), 0);

	if(total != 0)
		drive_robot(0.5, (float) (counter[0] - counter[2]) / total);
	else drive_robot(0, 0);
}

int main(int argc, char** argv) {
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
