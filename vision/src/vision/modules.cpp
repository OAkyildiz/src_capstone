/*
 * modules.cpp
 *
 *  Created on: Oct 12, 2016
 *      Author: oakyildiz
 */

#include "vision/modules.h"
#include "ros/ros.h"

int THRESHOLD = 10;
using namespace vision;
using namespace cv;


/////////////
/* VisionMoudel: Base cass*/
VisionModule::VisionModule(std::string name)
{
	namedWindow(window_name, 1);
	startWindowThread();


}

VisionModule::~VisionModule(){
	destroyWindow(window_name);
}

void VisionModule::getFrame(Mat img_in) {
	input = img_in;
}

///////////
/* LightModule: Light Detection Module*/

LightModule::LightModule(std::string name, short int t):
					VisionModule(name),
					threshold(t),slider(t),
					max(0), r(0),
					color(0,0,0)
{
	createTrackbar( "threshold", window_name, &slider,255,LightModule::onTrackbar, this);

}

void LightModule::doVision(){
	//inRange(input, cv::Scalar(0, 0, 0), cv::Scalar(180, 180, 180), colored);
	cvtGray(Size(3,3), 2, true);
	cv::threshold(grayscale, grayscale,slider,0,THRESH_TOZERO_INV);
	//seperateChannels(input,output);

	double min;
	minMaxLoc(grayscale, &min, &max, &min_pt, &max_pt);

	int r=0;
	if (max > threshold){
		// dynamic radius
		r = max / 10 + 2;
		Vec3b pixel = input.at<Vec3b>(max_pt);
		color = Scalar(pixel.val[0]-r, pixel.val[1]-r, pixel.val[2]-r);
		// constant radius
		// int r=10;


		circle(output, max_pt, r,  color, 3, 8, 0);
	}
	else{
		color = Scalar(0,0,0);
		r = 0;

	}
	present();
}


void LightModule::present(){
	imshow(window_name, grayscale);
    //waitKey(5);

	// debug
	// color
	ROS_INFO("val: %.0f  at (%d, %d)	",max, max_pt.x, max_pt.y);
	ROS_INFO("r: %d    BGR:(%.0f, %.0f, %.0f) \n",r/2 , color[0], color[1], color[2] );
	// exit on key
	//if(waitKey(30) >= 0) break;
}

void LightModule::cvtGray(Size size, double sigma, bool blur){
	cvtColor(input, grayscale, CV_RGB2GRAY);
	if (blur)GaussianBlur(grayscale, grayscale, size,sigma, sigma);
}

void LightModule::seperateChannels(Mat in, Mat out){
	// create local Mats
	Mat hsv_img;
	Mat red1,red2;
	Mat mask_med,mask;
	// convert colors
	cvtColor(in, hsv_img, COLOR_BGR2HSV);

	//red
	inRange(hsv_img, HSV_RED_LOW,HSV_RED_HIGH, red1);
	inRange(hsv_img, HSV_RED_2_LOW,HSV_RED_2_HIGH, red2);
	bitwise_or(red1,red2,red_ch);

	//blue
	inRange(hsv_img,HSV_BLUE_LOW, HSV_BLUE_HIGH, blue_ch);

	//green
	inRange(hsv_img, HSV_GREEN_LOW,HSV_GREEN_HIGH, green_ch);

	//finalize
	bitwise_or(red_ch,blue_ch, mask_med);
	bitwise_or(mask_med, green_ch, mask);
	//bitwise_and(hsv_img,mask,out);


}



void LightModule::onTrackbar(int val, void* ptr){
	LightModule* mod = (LightModule*)(ptr);
	mod->setThreshold(val);
}
//////////////
/* ObjectMoudel: Object recognition module*/
ObjectModule::ObjectModule(std::string name):
					VisionModule(name)
{
}

void ObjectModule::doVision(){}
void ObjectModule::present(){}


