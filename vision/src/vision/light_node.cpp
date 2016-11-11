/*
 * vision_node.cpp
 *
 *  Created on: Oct 3, 2016
 *      Author: oakyildiz, Ozan Akyıldız
 *		 Email: oakyildiz@wpi.edu
 *
 */

#include <ros/ros.h>
#include "vision/vision.h"


using namespace vision;

int main( int argc, char** argv ) {

 LightModule* light_detector =  new LightModule("stream",170);
 VisionNode* node = new VisionNode(light_detector, argc, argv);
//


 if(node->init("light_detection")){
	 node->nh_->param("threshold",THRESHOLD,THRESHOLD);
	 light_detector->setThreshold(THRESHOLD);
	 int t = light_detector->getThreshold();
	 ROS_INFO("threshold: %d \n", t);
	 node->run();
 	 return 0;
}
 else
	 return 1;
}

