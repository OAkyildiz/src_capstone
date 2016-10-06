/*
 * vision_node.h
 *
 *  Created on: Oct 4, 2016
 *      Author: oakyildiz, Ozan Akyıldız
 *		 Email: oakyildiz@wpi.edu
 *
 */

#ifndef INCLUDE_VISION_NODE_H_
#define INCLUDE_VISION_NODE_H_

#include <ros/ros.h>
#include <cv_bridge/rgb_colors.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include "opencv2/opencv.hpp"

namespace vision {

class VisionNode {
public:
	VisionNode( int argc, char** argv );
	virtual ~VisionNode(){}

	ros::NodeHandle nh;
private:

	ros::NodeHandle pnh;
	ros::Subscriber main_camera_sub;
	ros::Subscriber secondary_camera_Sub;

	ros::Publisher something_pub;

	bool visionCallback(const sensor_msgs::ImageConstPtr& msgr);
	bool publish();
	bool subscribe();
	int convertImage(const sensor_msgs::ImageConstPtr image);
	bool placeholder_do_light_stuff();

};


class VisionModule{

};
} /* namespace vision */

#endif /* INCLUDE_VISION_NODE_H_ */
