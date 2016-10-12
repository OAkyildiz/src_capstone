/*
 * light_detection_node.cpp
 *
 *  Created on: Oct 3, 2016
 *      Author: oakyildiz, Ozan Akyıldız
 *		 Email: oakyildiz@wpi.edu
 *
 */

#include "vision/vision.h"

namespace vision{
VisionNode::VisionNode(int argc, char** argv):
			  init_argc(argc),
			  init_argv(argv),
			  RATE(30),
			  nh_(0),
			  pnh_(0)
{}

VisionNode::~VisionNode() {
	if(ros::isStarted()) {
		ros::shutdown(); // explicitly needed since we use ros::start();
		ros::waitForShutdown();
	}
	delete nh_,pnh_;
	wait();
}

bool VisionNode::init(const std::string &name) {
	ros::init(init_argc, init_argv, name);
	if ( ! ros::master::check() ) {
		ROS_ERROR("Cannot communicate with the ROSMASTER!");
		return false;
	}
	setup();
	return true;
}


bool VisionNode::init(const std::string &name, const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,name);
	if ( ! ros::master::check() ) {
		ROS_ERROR("Cannot communicate with the ROSMASTER!");
		return false;
	}
	setup();
	return true;
}


void VisionNode::run() {
	ros::Rate node_rate(RATE); //talk about redundancy

	// Startup ROS spinner in background
	ros::AsyncSpinner spinner(2);
	spinner.start();

	int l=0;
	while( ros::ok( )){
		ros::spinOnce();
		node_rate.sleep();
		l++;
		ROS_INFO("working");
	}

}

bool VisionNode::visionCallback(const sensor_msgs::ImageConstPtr& image) {
	return convertImage(image);

}

int VisionNode::convertImage(const sensor_msgs::ImageConstPtr image) {

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

bool VisionNode::placeholder_do_light_stuff() {
	return 0;
}


bool VisionNode::setup() {
	nh_ = new ros::NodeHandle();
	pnh_ = new ros::NodeHandle("~");

	pnh_->param<double>("rate", RATE, 20);
	pnh_->param<bool>("is_stereo", IS_STEREO,20);
	/*subs and pubs*/
	return true;
}

} //namespace vision




