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

//TODO: separate modules.h, independent from cv_bridge

namespace vision {
enum NodeType {
	LIGHT =0, LIGHT_DETECTION =0, TASK1= 0,
	OBJECT = 1, DOOR = 1, BUTTON = 1, TASK2=1
};

bool IS_STEREO = false;
NodeType NODE_TYPE = 0;

std::string MAIN_CAMERA = "/main_camera/";
std::string SECONDARY_CAMERA = "/second_camera/";

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

using namespace cv;

class VisionModule{
public:
	VisionModule(std::string name);
	virtual ~VisionModule();

protected:
	virtual void doVision(){}
	virtual void present(){}

private:
	Mat colored;
	std::string window_name;



};

class LightModule: public VisionModule{

public:
	LightModule(std::string name,short int t);
	virtual ~LightModule(){}

	void doVision(){}
private:
	/* parameters*/
	short int threshold;

	/* members */
	double max;
	int r;
	Scalar color;
	Mat grayscale;
	cv::Point min_pt;
	cv::Point max_pt;
	void cvtGray(Size size,double sigma);
	void present();
};

class ObjectModule: public VisionModule{
public:
	ObjectModule(std::string name);
	virtual ~ObjectModule(){}

	virtual void doVision(){}

private:
	/* parameters */

	/* members */
	void present();

};
} /* namespace vision */

#endif /* INCLUDE_VISION_H_ */
