/*
 * transfrom_node.cpp
 *
 *  Created on: Dec 2, 2016
 *      Author: oakyildiz, Ozan Akyıldız
 *		 Email: oakyildiz@wpi.edu
 *
 */


#include <node/node.h>
#include <vision/modules.h>

#include <tf/transform_listener.h>

#include <geometry_msgs/PointStamped.h>
#include "vision/DetectedObject.h"

using namespace vision;


class TransformNode:public Node {
public:
	TransformNode(int argc, char** argv): Node(argc,argv),
	tf_(0){}
	virtual ~TransformNode(){}

	/* Members */

	/* Getters & Setters */

private:
	/* Members */
	tf::TransformListener* tf_;
	ros::Subscriber objects_sub_;

	ros::Publisher viz_points_pub;
	ros::Publisher led_pub_;
	ros::Publisher button_pub_;
	ros::Publisher door_pub_;

	/* member functions to customize node*/
	void setupParams(){}
	void setupCustom(){
			tf_ = new tf::TransformListener(*nh_);

			objects_sub_= nh_-> subscribe("/objects", 30, &TransformNode::objectCallback, this);

			led_pub_= nh_->advertise<geometry_msgs::PointStamped>("/leds", 30);
			button_pub_= nh_->advertise<geometry_msgs::PointStamped>("/button", 30);
			door_pub_= nh_->advertise<geometry_msgs::PointStamped>("/door", 30);

	}

	void operation(){}

	void objectCallback(const vision::DetectedObjectConstPtr& obj){
		//ROS_INFO("caught object");
		ros::Time detection_time=obj->header.stamp;

		tf::StampedTransform transform;
		//the topic name and string variables are set to LEDs as default
		std::string target_frame = "head";
		std::string topic = "/leds";
		ros::Publisher*  selected_pub_ptr =&led_pub_;
		//ROS_INFO("registering object");

		geometry_msgs::PointStamped* detected_point;
		detected_point->header.frame_id=obj->header.frame_id;
		detected_point->header.stamp=detection_time;
		//detected_point->header.stamp==ros::Time().now();
		detected_point->point=obj->point;
		uint8_t type = obj->type;
		//ROS_INFO("registered object");

		geometry_msgs::PointStamped point_out;
		
		//each case describes a dtection case of either LEDs or objects.
		switch(type){
		//red LED		
		case vision::RED_LED: 
			ROS_INFO("RGB:(1,0,0)");
			break;\
		//green LED
		case vision::GREEN_LED:
			ROS_INFO("RGB:(0,1,0)");
			break;
		//blue LED
		case vision::BLUE_LED:
			ROS_INFO("RGB:(0,0,1)");
			break;


			break;
		case vision::BUTTON:
		//set frame and button topic
			target_frame="pelvis";
			topic="/button";
			selected_pub_ptr=&button_pub_;

			break;
		//set frame and door topic
		case vision::DOOR_OPEN:
		case vision::DOOR_CLOSED:
			target_frame="pelvis";
			topic="/door";
			selected_pub_ptr=&button_pub_;
			break;
		}
		
		try {
			//ROS_INFO("try object");
			//There is a slight delay between joint states and points we detected and stamped
			//wait for transforms
			tf_->waitForTransform(target_frame,detected_point->header.frame_id,
			                              detection_time, ros::Duration(3));
			//transform point.
			tf_->transformPoint(target_frame, *detected_point, point_out);
			ROS_INFO("Detected obj (->head): (x:%.3f y:%.3f z:%.3f)\n",
		             point_out.point.x,
		             point_out.point.y,
		             point_out.point.z);
		}
		catch (tf::TransformException &ex){

			ROS_INFO ("Failure %s\n", ex.what()); //Print exception for TF error
		}
		//ROS_INFO("publish object");

		selected_pub_ptr->publish(point_out);
	}

};


//Run the node
int main(int argc, char** argv){
	TransformNode* tfnode = new TransformNode(argc,argv);
	tfnode->init("object_tf");
	tfnode->run();

	return 0;
}
