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

#include <string.h>

#include <ros/ros.h>
#include <ros/console.h>

#include <cv_bridge/rgb_colors.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

#include "opencv2/opencv.hpp"

namespace vision {

class VisionNode {
public:
	VisionNode(int argc, char** argv);
	virtual ~VisionNode();

	bool init(const std::string &name = "vision_node");
	bool init(const std::string &name, const std::string &master_url, const std::string &host_url);
	void run();


	/* Members */
	ros::NodeHandle* nh_;

	/* Getters & Setters */
	const ros::NodeHandle* get_nh() const {
			return nh_;
		}
private:

	/* Members */
	int init_argc;
	char** init_argv;

	ros::NodeHandle* pnh_;

	ros::Subscriber* sub_list[];
	ros::Publisher* pub_list[];


	ros::Subscriber main_camera_sub;
	ros::Subscriber secondary_camera_Sub;

	ros::Publisher something_pub;

	bool visionCallback(const sensor_msgs::ImageConstPtr& image);
	bool publish();
	bool subscribe();

	int convertImage(const sensor_msgs::ImageConstPtr image);
	bool placeholder_do_light_stuff();

	bool setup();

	double RATE = 60;
	bool IS_STEREO = true;

};


class VisionModule{
public:
	VisionModule();
	virtual ~VisionModule(){}
private:

};
} /* namespace vision */

#endif /* INCLUDE_VISION_NODE_H_ */
