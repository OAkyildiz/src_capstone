/*
 * light_detection_node.cpp
 *
 *  Created on: Oct 3, 2016
 *      Author: oakyildiz, Ozan Akyıldız
 *		 Email: oakyildiz@wpi.edu
 *
 */

#include "vision/vision.h"

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
	}


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
	std::string info="camera_info";

	if(IS_WEBCAM){
		IS_STEREO = false;	// for now, we override stereo
							// to false if webcam is used.
		CAMERA_NAMESPACE = "/usb_cam"; //TODO: change how this is handled.
		mono_camera_sub = it_-> subscribe(CAMERA_NAMESPACE + "/" + IMAGE_TYPE, 100, &VisionNode::visionCallback, this);
		mono_info_sub = nh_-> subscribe(CAMERA_NAMESPACE + "/" + info, 100, &VisionNode::camereInfoCallback, this);

	}
	else{
		left_camera_sub = it_-> subscribe(CAMERA_NAMESPACE + "/left/"+ IMAGE_TYPE, 100, &VisionNode::visionCallback, this);
		left_info_sub = pnh_-> subscribe(CAMERA_NAMESPACE + "/left/"+ info, 100, &VisionNode::camereInfoCallback, this);

		if(IS_STEREO){
			right_camera_sub = it_-> subscribe(CAMERA_NAMESPACE + "/right/"+ IMAGE_TYPE, 1000, &VisionNode::disparityCallback, this);
			right_info_sub = nh_-> subscribe(CAMERA_NAMESPACE + "/right/"+ info, 100, &VisionNode::camereInfoCallback, this);

		}
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
		module->show();
}

/* Operations */
void VisionNode::visionCallback(const sensor_msgs::ImageConstPtr& image) {
	//TODO: pass the setter method and object pointer to the helper method instead.
	if( convertImage(image, STEREO_LEFT) == 0){
		int e1 = getTickCount();
		module->doVision();
		int e2 = getTickCount();
		safe = true;
		//ROS_INFO("Operation time: %.3f \n",(e2 - e1)/ getTickFrequency());

	}
	else safe=false;


}
void VisionNode::disparityCallback(const sensor_msgs::ImageConstPtr& image) {
	convertImage(image, STEREO_RIGHT);

}

void vision::VisionNode::camereInfoCallback (const sensor_msgs::CameraInfoConstPtr& cam_info) {
}

int VisionNode::convertImage(const sensor_msgs::ImageConstPtr image, int sel) {

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




