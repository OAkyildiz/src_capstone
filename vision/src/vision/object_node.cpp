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

 VisionModule* obj_detector =  new ObjectModule("stream");
 VisionNode* node = new VisionNode(obj_detector, argc, argv);

 node->init("vision_node");
 node->run();

  return 0;
}

