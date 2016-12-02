/*
 * footstep_node.cpp
 *
 *  Created on: , 2016
 *      Author: ,  
 *		 Email: @wpi.edu
 *
 */

#include "footstep_planner/footstep_node.h"

int main( int argc, char** argv ) {

 FootstepPlannerNode node = new FootstepPlannerNode(argc, argv);



 node->init("footstep_node");
 node->run();


  return 0;
}
