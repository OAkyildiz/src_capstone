/*
 * footstep_node.h
 *
 *  Created on: November 28, 2016
 *      Author: smcgovern, Sean 
 *		 Email: @wpi.edu
 *
 */


#ifndef INCLUDE_NODE_H_
#define INCLUDE_NODE_H_

#include <string.h>
#include <vector.h>

#include <ros/ros.h>
#include <ros/console.h>

// include message types from ihmc_messages
//probably i.e. #include "ihmc_msgs/FootstepStatusRosMessage.h"
// and these:
//	 FootstepDataListRosMessage
//	 FootstepDataRosMessage

struct Leg{
	double lower;
	double upper;
	double total;
	double hip_height;
}

class FootstepPlannerNode: public Node {
public:
	FootstepPlannerNode(int argc, char** argv);
	virtual ~FootstepPlannerNode(){}




	/* Members */
	
	Leg valkyrie_leg;
	/* Getters & Setters */
	

protected:
	

private:
	/

	/* Members */
	ros::Subscriber goal_sub;	
	ros::Publisher steps_pub;

	//add a vector<Footsteps> member; //gotta look up Type form doc.
	
	/* member functions to customize node*/
	
	void setupParams();
	void setupCustom();
	void operation();

	void calculateStep();
	void updateGoal();
	/*hash map of values*/
	//HashMap<std::string, double> params;
	//static double RATE = 60;
};
