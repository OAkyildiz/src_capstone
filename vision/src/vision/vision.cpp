/*
 * light_detection_node.cpp
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

 VisionNode* node = new VisionNode(argc, argv);



 node->init("vision_node");
 node->run();
   //detection stuff here

  return 0;
}

