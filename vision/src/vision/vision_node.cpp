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
/* Node */
VisionNode::VisionNode(VisionModule* module, int argc, char** argv):
	Node(argc,argv),
	module(module)
	{}


void VisionNode::setupParams() {
	pnh_->param<bool>("is_stereo", IS_STEREO,true);
	pnh_->param<NodeType>("node_type", NODE_TYPE,LIGHT);
	pnh_->param<std::string>("main_camera",MAIN_CAMERA,"/usb_cam/");
	pnh_->param<std::string>("secondary_camera", SECONDARY_CAMERA,"/secondary_camera/");


	/*subs and pubs*/
}

virtual void setupCustom(){

}

void VisionNode::operation() {
// operations here
	module->doVision();
	module->present();
}

/* Operations */
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



} //namespace vision




