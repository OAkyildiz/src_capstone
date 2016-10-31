/*
 * node.cpp
 *
 *  Created on: Oct 24, 2016
 *      Author: oakyildiz, Ozan Akyıldız
 *		 Email: oakyildiz@wpi.edu
 *
 */


#include "node/node.h"

Node::Node(int argc, char** argv):
			  init_argc(argc),
			  init_argv(argv),
			  nh_(0),
			  pnh_(0)
{}

Node::~Node() {
	if(ros::isStarted()) {
		ros::shutdown(); // explicitly needed since we use ros::start();
		ros::waitForShutdown();
	}
	delete nh_,pnh_;
	wait();
}

bool Node::init(const std::string &name) {
	ros::init(init_argc, init_argv, name);
	if ( ! ros::master::check() ) {
		ROS_ERROR("Cannot communicate with the ROSMASTER!");
		return false;
	}
	setup();
	return true;
}


bool Node::init(const std::string &name, const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,name);
	if ( ! ros::master::check() ) {
		ROS_ERROR("Cannot communicate with the ROSMASTER!");
		return false;
	}
	setup();
	return true;
}

bool Node::setup() {
		nh_ = new ros::NodeHandle();
		pnh_ = new ros::NodeHandle("~");

		pnh_->param<double>("rate", RATE, 20);
		setupParams();
		setupCustom();
		/*subs and pubs*/

		return true;
	}

void Node::run() {
	ros::Rate node_rate(RATE); //talk about redundancy

	// Startup ROS spinner in background
	ros::AsyncSpinner spinner(2);
	spinner.start();

	int l=0;
	while( ros::ok( )){
		ros::spinOnce();
		node_rate.sleep();
		operation();
		l++;
		ROS_DEBUG("node spinning");
	}

}

