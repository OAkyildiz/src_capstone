#include "footstep_node.h"

FootstepPlannerNode::FootstepPlannerNode(int argc, char** argv):
Node(arfc,argv){
}

void FootstepPlannerNode::setupParams(){
 /* Read in Ros Params */



}


void FootstepPlannerNode::setupCustom(){
 /* Initialize subscribers, publishers and transform listener here*/
 /* transform listener tutorials: google "ros tf" */




}


void FootstepPlannerNode::operations(){

/* Main algorithim goes here*/


    
double L1 = .8342; 
double L2 = .0609;
double L3 = L1+L2;

H = .71736;

A1 = acos(H/L3);

R = sin(A1)*L3;




x = cos(arc)*R-.040204;
y = sin(arc)*R+.1070573;


}

void FootstepPlannerNode::calculateStep(){
}

void FootstepPlannerNode::updateGoal(){
}
