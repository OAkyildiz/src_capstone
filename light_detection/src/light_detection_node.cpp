/*
 * light_detection_node.cpp
 *
 *  Created on: Oct 3, 2016
 *      Author: oakyildiz, Ozan Akyıldız
 *		 Email: oakyildiz@wpi.edu
 *
 */

#include <ros/ros.h>

int main( int argc, char** argv ) {
  ros::init(argc, argv, "light_detector");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  double stream_rate;
  pnh.param<double>("stream_rate", stream_rate, 60);


  // Startup ROS spinner in background
  ros::AsyncSpinner spinner(2);
  spinner.start();

  if(!ros::ok()) {
    ROS_FATAL("Failed to initialize node");
    return 1;
  }
  ros::Rate rate(stream_rate);

  while (ros::ok()) {
    ros::Duration dt;
    ros::Time now;

   //detection stuff here

    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}


VisionNode::VisionNode(){

}

VisionNode::~VisionNode(){

}

VisionNode::
