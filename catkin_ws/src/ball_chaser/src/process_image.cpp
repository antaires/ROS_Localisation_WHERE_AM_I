#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
// Image processing
#include <opencv/cv.h> //TODO correct includes
#include <highgui.h>



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
    // Use image processing to locate circles (steer towards balls of any color)
    Mat gray;
    cvtColor(img, gray, CV_BGR2GRAY);
    // Smooth to avoid false detection
    GaussianBlur(gray, gray, Size(9, 9), 2, 2);
    std::vector<Vec3f> circles;
    HoughCircles(gray, circles, CV_HOUGH_GRADIENT, 2, gray->rows/4, 200, 100);

    if(circles.size() == 0)
    {
        drive_robot(0.0f, 0.0f); 
    }
    else
    {
        //take first circle 
        const float y = circles[0][1];

        float linear_x = 0.0f;
        float angular_z = 0.0f;
        const int min = (img.step+1) / 4;
        const int max = img.step - min; 

        if(y < min)
           angular_z = 0.1f;
        else if (y > max) 
           angular_z = -0.1f;
        else
           linear_x = 0.5f;

        drive_robot(linear_x, angular_z);
        ros::Duration(3).sleep();
    }

    /* CODE TO STEER TOWARDS WHITE PIXEL ASSUMED TO BE A BALL
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
    */
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
