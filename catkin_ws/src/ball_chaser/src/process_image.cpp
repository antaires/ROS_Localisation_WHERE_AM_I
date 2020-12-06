#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    ROS_INFO_STREAM("Moving the robot to input linear.x and angular.z");

    // Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    if(!client.call(srv))
        ROS_ERROR("Failed to drive robot");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    const int white_pixel = 255;

    // Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
    float linear_x = 0.0f;
    float angular_z = 0.0f;
    const int min = (img.step+1) / 4;
    const int max = img.step - min;  
    for(int i = 0; i < img.height * img.step; ++i)
    {
        if(img.data[i] == white_pixel)
        {
		ROS_INFO_STREAM("WHITE PIXEL FOUND - " + std::to_string(i));
		ROS_INFO_STREAM("MIN - " + std::to_string(min));
		ROS_INFO_STREAM("Max - " + std::to_string(max));
            const int y = i % img.step;
            if(y < min)
               angular_z = 0.1f;
            else if (y > max) 
               angular_z = -0.1f;
            else
               linear_x = 0.5f;
	
            drive_robot(linear_x, angular_z);
            ros::Duration(3).sleep();
            return;
        }
    }

    //stops robot if no white visible
    drive_robot(linear_x, angular_z); 
    ros::Duration(3).sleep();
}

int main(int argc, char** argv)
{
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
