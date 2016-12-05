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
	ros::Publisher target_pub;

	/* member functions to customize node*/
	void setupParams(){}
	void setupCustom(){}
	void operation(){}

	void objectCallback(const vision::DetectedObjectConstPtr& obj){

		tf::StampedTransform transform;
		geometry_msgs::PointStamped* detected_point;
		detected_point->header.frame_id=obj->frame_id;
		detected_point->header.stamp=ros::Time(0);
		detected_point->point=obj->point;
		vision::Type type = obj->type;

		geometry_msgs::PointStamped point_out;
		try {
			tf_.transformPoint("head", *detected_point, point_out);
			printf("Detected obj (->head): (x:%f y:%f z:%f)\n",
		             point_out.point.x,
		             point_out.point.y,
		             point_out.point.z);
		}
		catch (tf::TransformException &ex){
			printf ("Failure %s\n", ex.what()); //Print exception which was caught
		}

		switch(type){
		case vision::RED_LED:
		case vision::GREEN_LED:
		case vision::BLUE_LED:
			break;
		case vision::BUTTON:
		case vision::DOOR_OPEN:
		case vision::DOOR_CLOSED:
			break;
		}


	}
};



int main(int argc, char** argv){
	TransformNode* tfnode = new TransformNode(argc,argv);
	tfnode->init("object_tf");
	tfnode->run();

	return 0;
}
