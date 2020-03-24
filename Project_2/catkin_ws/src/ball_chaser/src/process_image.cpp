#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/imgproc.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <image_transport/image_transport.h>
#include <list>

namespace enc = sensor_msgs::image_encodings;

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{

    ROS_INFO_STREAM("Driving the robot towards the ball");

    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = (float)lin_x;
    srv.request.angular_z = (float)ang_z;

    if (!client.call(srv))
            ROS_ERROR("Failed to call service command_robot");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::ImageConstPtr& msg )
{

	// Convert to rgb
	cv_bridge::CvImageConstPtr cv_ptr;
	cv_ptr = cv_bridge::toCvShare(msg, enc::BGR8);
	cv::Mat src_gray;
	cvtColor( cv_ptr->image, src_gray, CV_BGR2GRAY );

	// Reduce the noise so we avoid false circle detection
	cv::GaussianBlur( src_gray, src_gray, cv::Size(9, 9), 2, 2);

	// Apply the hough transform to find the circles
	std::vector<cv::Vec3f> circles;
	cv::HoughCircles( src_gray, circles, CV_HOUGH_GRADIENT, 1, src_gray.rows/8, 70, 35, 0, 0);

	// Draw the circles detected
	for( size_t i = 0; i < circles.size(); i++ ) {
		cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
		int radius = cvRound(circles[i][2]);

		// circle center
		cv::circle( src_gray, center, 3, cv::Scalar(0,255,0), -1, 8, 0);

		// circle outline
		cv::circle( src_gray, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
	}

	// Show your results
	cv::namedWindow( "Hough Circle Transform Demo", CV_WINDOW_AUTOSIZE);
	cv::imshow( "Hough Circle Transform Demo", src_gray );

	cv::waitKey(10);

	if(circles.size() != 1) {
		ROS_WARN_STREAM(circles.size() << " circles detected");
		ROS_WARN_STREAM("Finding for a ball to play with...");
		drive_robot(0,0.5);
	}
        else
	{	
	        int left = src_gray.cols/3;
		int centre = left*2;
	
		if (circles[0][0] < left)
		{ ROS_INFO_STREAM("Moving towards left");
	          drive_robot(0.5, 1.2);
		}
	        else if(circles[0][0] < centre)
		{ ROS_INFO_STREAM("Moving straight");
	          drive_robot(0.5, 0);
		}
	        else
		{ ROS_INFO_STREAM("Moving towards right"); 
	          drive_robot(0.5, -1.2);
		}
	 }	
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

