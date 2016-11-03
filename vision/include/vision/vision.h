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

#include <image_transport/image_transport.h>
#include <cv_bridge/rgb_colors.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>


//TODO: separate modules.h, independent from cv_bridge
extern bool IS_STEREO;
extern bool IS_WEBCAM;
//NodeType NODE_TYPE = 0;
//std::string NODE_TYPE;

extern std::string CAMERA_NAMESPACE;
extern std::string IMAGE_TYPE;




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
	image_transport::ImageTransport* it_;

	// Possible TODO: Seperate mono and stereo nodes;
	image_transport::Subscriber mono_camera_sub;
	ros::Subscriber mono_info_sub;


	image_transport::Subscriber left_camera_sub;
	image_transport::Subscriber right_camera_sub;

	ros::Subscriber left_info_sub;
	ros::Subscriber right_info_sub;

	ros::Publisher something_pub;

	//static bool IS_STEREO = true;


	void setupParams();
	void setupCustom();
	void operation();

	void visionCallback(const sensor_msgs::ImageConstPtr& image);
	void disparityCallback(const sensor_msgs::ImageConstPtr& image);

	void camereInfoCallback(const sensor_msgs::CameraInfoConstPtr& cam_info);


	bool publish();
	bool subscribe();

	int convertImage(const sensor_msgs::ImageConstPtr image);

};

} /* namespace vision */

#endif /* INCLUDE_VISION_H_ */
