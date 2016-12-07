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
		//TODO: move this to operatoion to constantly pub?
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
		

		switch(type){
		case vision::RED_LED:
			ROS_INFO("RGB:(1,0,0)");
			break;
		case vision::GREEN_LED:
			ROS_INFO("RGB:(1,0,0)");
			break;
		case vision::BLUE_LED:
			ROS_INFO("RGB:(1,0,0)");
			break;


			//led publish
			break;
		case vision::BUTTON:
			target_frame="pelvis";
			topic="/button";
			selected_pub_ptr=&button_pub_;

			break;
			//set frame
		case vision::DOOR_OPEN:
		case vision::DOOR_CLOSED:
			target_frame="pelvis";
			topic="/door";
			selected_pub_ptr=&button_pub_;
			break;
		}
		
		try {
			//ROS_INFO("try object");

			tf_->waitForTransform(target_frame,detected_point->header.frame_id,
			                              detection_time, ros::Duration(3));
			tf_->transformPoint(target_frame, *detected_point, point_out);
			ROS_INFO("Detected obj (->head): (x:%f y:%f z:%f)\n",
		             point_out.point.x,
		             point_out.point.y,
		             point_out.point.z);
		}
		catch (tf::TransformException &ex){

			ROS_INFO ("Failure %s\n", ex.what()); //Print exception which was caught
		}
		//ROS_INFO("publish object");

		selected_pub_ptr->publish(point_out);
	}

};



int main(int argc, char** argv){
	TransformNode* tfnode = new TransformNode(argc,argv);
	tfnode->init("object_tf");
	tfnode->run();

	return 0;
}
