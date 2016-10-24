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
VisionNode::VisionNode(int argc, char** argv):
		Node(argc,argv),
		IS_STEREO(true)
	{}

void VisionNode::operation() {
// operations here

}

void VisionNode::setup_params() {
	pnh_->param<bool>("is_stereo", IS_STEREO,20);
	/*subs and pubs*/
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




