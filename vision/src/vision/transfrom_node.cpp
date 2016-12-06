/*
 * transfrom_node.cpp
 *
 *  Created on: Dec 2, 2016
 *      Author: oakyildiz, Ozan Akyıldız
 *		 Email: oakyildiz@wpi.edu
 *
 */


#include <node/node.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>


class TransformNode:public Node {
public:
	TransformNode(int argc, char** argv): Node(argc,argv){}
	virtual ~TransformNode(){}

	/* Members */

	/* Getters & Setters */

private:
	/* Members */
	tf::TransformListener tf_;
	ros::Subscriber objects_sub_;

	ros::Publisher viz_points_pub;
	ros::Publisher led_pub_;
	ros::Publisher button_pub_;
	ros::Publisher door_pub_;

	/* member functions to customize node*/
	void setupParams(){}
	void setupCustom(){
			
			objects_sub_= ;
			object_pub_= nh_->advertise<geometry_msgs::PointStamped>("/objects", 30);
	
	}
	void operation(){}

	void objectCallback(const vision::DetectedObjectConstPtr& obj){

		tf::StampedTransform transform;
		std::string target_frame;
		std::string topic;

		ros::Publisher*  selected_pub_ptr;
		geometry_msgs::PointStamped* detected_point;
		detected_point->header.frame_id=obj->frame_id;
		detected_point->header.stamp=ros::Time(0);
		detected_point->point=obj->point;
		vision::Type type = obj->type;

		geometry_msgs::PointStamped point_out;
		

		switch(type){
		case vision::RED_LED:
		case vision::GREEN_LED:
		case vision::BLUE_LED:
			target_frame="head"
			//led publish
			break;
		case vision::BUTTON:
			target_frame="pelvis"
			topic="/button"
			//set frame
		case vision::DOOR_OPEN:
		case vision::DOOR_CLOSED:
			target_frame="pelvis";
			topic="/button";
			selected_pub_ptr=button_pub*;
			break;
		}
		
		try {
			tf_.transformPoint(target_frame, *detected_point, point_out);
			printf("Detected obj (->head): (x:%f y:%f z:%f)\n",
		             point_out.point.x,
		             point_out.point.y,
		             point_out.point.z);
		}
		catch (tf::TransformException &ex){
			printf ("Failure %s\n", ex.what()); //Print exception which was caught
		}
		
	}
};



int main(int argc, char** argv){
	TransformNode* tfnode = new TransformNode(argc,argv);
	tfnode->init("object_tf");
	tfnode->run();

	return 0;
}
