/*
 * node_main.cpp
 *
 *  Created on: Oct 3, 2016
 *      Author: oakyildiz, Ozan Akyıldız
 *		 Email: oakyildiz@wpi.edu
 *
 */

#include "node/node.h"

int main( int argc, char** argv ) {

 Node* node = new Node(argc, argv);



 node->init("node");
 node->run();
   //detection stuff here

  return 0;
}
