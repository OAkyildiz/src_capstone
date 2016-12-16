/*
 * light_detection_node.cpp
 *
 *  Created on: Oct 3, 2016
 *      Author: oakyildiz, Ozan Akyıldız
 *		 Email: oakyildiz@wpi.edu
 *
 */

#include "vision/vision.h"

// parameters for hardware congif.
bool IS_STEREO = true;
bool IS_WEBCAM = false;
bool safe = false;

std::string CAMERA_NAMESPACE = "/multisense/camera";
std::string IMAGE_TYPE;

using namespace vision;
/* Node */
VisionNode::VisionNode(VisionModule* module, int argc, char** argv):
			Node(argc,argv),
			module(module),
			it_(0)
{
	//module->setParent(this);
	}

//Setup ros parameters from parameter server
void VisionNode::setupParams() {
	it_ = new image_transport::ImageTransport(*nh_);
	pnh_->param<bool>("stereo", IS_STEREO, IS_STEREO); // lets  default param values to
	pnh_->param<bool>("webcam", IS_WEBCAM,IS_WEBCAM);  // source ode's variable valeus

	pnh_->param<std::string>("image",IMAGE_TYPE,"image_raw");

	//pnh_->param<NodeType>("node_type", NODE_TYPE,LIGHT);
	pnh_->param<std::string>("camera_namespace",CAMERA_NAMESPACE,"/multisense/camera");

	/*overrides*/

}
void VisionNode::setupCustom(){
if(IMAGE_TYPE != "image_raw") IMAGE_TYPE = "image_raw/" + IMAGE_TYPE;
	/*subs and pubs*/
	target_px_error_pub = nh_->advertise<vision::IntTuple>("/target_px_error", 30);
	object_pub = nh_->advertise<vision::DetectedObject>("/objects", 30);
	std::string info = "camera_info";
	//set the correct subscribers depending on camera type
	if(IS_WEBCAM){
		IS_STEREO = false;	// for now, we override stereo
							// to false if webcam is used.
		CAMERA_NAMESPACE = "/usb_cam"; //TODO: change how this is handled.
		mono_camera_sub = it_-> subscribe(CAMERA_NAMESPACE + "/" + IMAGE_TYPE, 100, &VisionNode::visionCallback, this);
		mono_info_sub = nh_-> subscribe(CAMERA_NAMESPACE + "/" + info, 100, &VisionNode::camereInfoCallback, this);

	}
	else{
		left_camera_sub = it_-> subscribe(CAMERA_NAMESPACE + "/left/"+ IMAGE_TYPE, 100, &VisionNode::visionCallback, this);

		if(IS_STEREO){
			right_camera_sub = it_-> subscribe(CAMERA_NAMESPACE + "/right/"+ IMAGE_TYPE, 1000, &VisionNode::disparityCallback, this);
			right_info_sub = nh_-> subscribe(CAMERA_NAMESPACE + "/right/"+ info, 100, &VisionNode::camereInfoCallback, this);
		}
		else
				left_info_sub = pnh_-> subscribe(CAMERA_NAMESPACE + "/left/"+ info, 100, &VisionNode::camereInfoCallback, this);


		}
}



/*READ*/
// For the following two functions, running with each callback or with node rate is possible.
// Decide if either presenting or image ops are worth operating at node's rate.
// If so, put them on operation() accordingly with proper flags and null checks.
void VisionNode::operation() {
	// operations here
	//module->doVision();
	if(safe)
		int e1 = getTickCount();
		module->doVision();
		int e2 = getTickCount();
		//ROS_INFO("Operation time: %.3f \n",(e2 - e1)/ getTickFrequency());
		publish();
		module->show();
}
bool VisionNode::publish(){
	//publish the pixeldistance from center for neck controller
	if (module->sel!=NONE){
		Point err = module->getPxError();
		IntTuple err_msg;
		err_msg.x=err.x;
		err_msg.y=err.y;
		target_px_error_pub.publish(err_msg);

	}
	//create the dtected object only if module set it's state to OUTPUT
	if (module->sel==OUTPUT){
		DetectedObject obj_msg;
		obj_msg.header.frame_id = module->getCam().frame_id;
		obj_msg.header.stamp=ros::Time().now();

		Point3d pt = module->getLocation();
		obj_msg.point.x = pt.x;
		obj_msg.point.y = pt.y;
		obj_msg.point.z = pt.z;
		obj_msg.type = module->getType();

		object_pub.publish(obj_msg);

		module->sel=DETECTED;
		return true;
	}
	else
		return false;
}
/* Operations */
void VisionNode::visionCallback(const sensor_msgs::ImageConstPtr& image) {
	//TODO: pass the setter method and object pointer to the helper method instead.
	if( convertImage(image, STEREO_LEFT) == 0){

		safe = true;

	}
	else safe = false;


}
// 2nd camera calllback
void VisionNode::disparityCallback(const sensor_msgs::ImageConstPtr& image) {
	convertImage(image, STEREO_RIGHT);

}
// Maybe assign fileds 1 by 1 in loadCamera
void vision::VisionNode::camereInfoCallback (const sensor_msgs::CameraInfoConstPtr& cam_info) {
	string  frame_id=cam_info->header.frame_id;
	boost::array<double, 12ul> P = cam_info->P;
	//ROS_INFO("%f, %f, %f, %f, %f",P[0],P[5],P[2],P[6],P[3]);
	if(!(module->camIsSet())){
		Camera cam = {true,cam_info->height,cam_info->width,P[0],P[5],P[2],P[2],P[6],0,P[3], frame_id };
		module->loadCamera(cam);
		ROS_INFO("loaded camera");
	}
}
//Convert ROS images to OpenCV's cv::Mat
int VisionNode::convertImage(const sensor_msgs::ImageConstPtr image, int sel) {
	// except in case message is blank
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
		if ( sel == STEREO_LEFT )
			module->loadFrame(cv_ptr->image);
		else if (sel == STEREO_RIGHT )
			module->loadFrame_Stereo(cv_ptr->image);
		return 0;
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return 1;
	}
}
//} //namespace vision




