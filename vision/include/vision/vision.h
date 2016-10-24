/*
 * vision_node.h
 *
 *  Created on: Oct 4, 2016
 *      Author: oakyildiz, Ozan Akyıldız
 *		 Email: oakyildiz@wpi.edu
 *
 */

#ifndef INCLUDE_VISION_H_
#define INCLUDE_VISION_H_

#include <string.h>

#include "node/node.h"

#include <cv_bridge/rgb_colors.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

#include "opencv2/opencv.hpp"

namespace vision {

class VisionNode: public Node {
public:
	VisionNode(int argc, char** argv);
	virtual ~VisionNode(){}

private:

	/* Members */

	ros::Subscriber main_camera_sub;
	ros::Subscriber secondary_camera_Sub;

	ros::Publisher something_pub;

	//static bool IS_STEREO = true;
	bool IS_STEREO;

	virtual void operation();
	virtual void setup_params();

	bool visionCallback(const sensor_msgs::ImageConstPtr& image);
	bool publish();
	bool subscribe();

	int convertImage(const sensor_msgs::ImageConstPtr image);

};


class VisionModule{
public:
	VisionModule();
	virtual ~VisionModule(){}
private:

};
} /* namespace vision */

#endif /* INCLUDE_VISION_H_ */
