/*
 * light_detection_node.cpp
 *
 *  Created on: Oct 3, 2016
 *      Author: oakyildiz, Ozan Akyıldız
 *		 Email: oakyildiz@wpi.edu
 *
 */

#include "../include/vision_node.h"

namespace vision{
VisionNode::VisionNode( int argc, char** argv ){

}

bool VisionNode::visionCallback(const sensor_msgs::ImageConstPtr& msgr) {
	return true;

}

int VisionNode::convertImage(const sensor_msgs::ImageConstPtr image){

	        cv_bridge::CvImagePtr cv_ptr;
	          try
	            {
	              cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
	              return 0;
	            }
	            catch (cv_bridge::Exception& e)
	            {
	              ROS_ERROR("cv_bridge exception: %s", e.what());
	              return 1;
	            }
}

bool VisionNode::placeholder_do_light_stuff(){
	return 0;
}

}
int main( int argc, char** argv ) {
	ros::init(argc, argv, "light_detector");
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	double stream_rate;
	pnh.param<double>("stream_rate", stream_rate, 60);


	// Startup ROS spinner in background
	ros::AsyncSpinner spinner(2);
	spinner.start();

	if(!ros::ok()) {
		ROS_FATAL("Failed to initialize node");
		return 1;
	}
	ros::Rate rate(stream_rate);

	while (ros::ok()) {
		ros::Duration dt;
		ros::Time now;

		//detection stuff here

		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}




