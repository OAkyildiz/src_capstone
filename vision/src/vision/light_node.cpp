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

 VisionModule* light_detector =  new LightModule("stream",170);
 VisionNode* node = new VisionNode(light_detector, argc, argv);
//


 if(node->init("light_detection")){
	 node->run();
 	 return 0;
}
   //detection stuff here
 else
	 return 0;
}

