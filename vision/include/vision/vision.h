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
#include "vision/modules.h"

#include <cv_bridge/rgb_colors.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>


//TODO: separate modules.h, independent from cv_bridge
extern bool IS_STEREO;
//NodeType NODE_TYPE = 0;
//std::string NODE_TYPE;

extern std::string MAIN_CAMERA;
extern std::string SECONDARY_CAMERA;

namespace vision {
//enum NodeType {
//	LIGHT =0, LIGHT_DETECTION =0, TASK1= 0,
//	OBJECT = 1, DOOR = 1, BUTTON = 1, TASK2=1
//};



class VisionNode: public Node {
public:
	VisionNode(VisionModule* module, int argc, char** argv);
	virtual ~VisionNode(){}

private:

	/* Members */
	VisionModule* module;
	ros::Subscriber main_camera_sub;
	ros::Subscriber secondary_camera_Sub;

	ros::Publisher something_pub;

	//static bool IS_STEREO = true;


	virtual void setupParams();
	virtual void setupCustom();
	virtual void operation();

	bool visionCallback(const sensor_msgs::ImageConstPtr& image);
	bool publish();
	bool subscribe();

	int convertImage(const sensor_msgs::ImageConstPtr image);

};

} /* namespace vision */

#endif /* INCLUDE_VISION_H_ */
