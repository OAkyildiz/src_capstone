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

VisionModule::VisionModule(std::string name)
{
	namedWindow(window_name, 1);
	startWindowThread();

}

VisionModule::~VisionModule(){
	destroyWindow(window_name);
}

/* LightModule: Light Detection Module*/

LightModule::LightModule(std::string name, short int t):
					VisionModule(name),
					threshold(t),
					max(0), r(0),
					color(0,0,0)
{
}

void LightModule::doVision(){
	//inRange(colored, cv::Scalar(0, 0, 0), cv::Scalar(180, 180, 180), colored);
	cvtGray(Size(7,7), 2);
	double min;
	cv::minMaxLoc(grayscale, &min, &max, &min_pt, &max_pt);

	int r=0;
	if (max > threshold){
		// dynamic radius
		r = max / 10 + 2;
		Vec3b pixel = colored.at<Vec3b>(max_pt);
		color = Scalar(pixel.val[0]-r, pixel.val[1]-r, pixel.val[2]-r);
		// constant radius
		// int r=10;


		circle(grayscale, max_pt, r,  color, 3, 8, 0);
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

void LightModule::cvtGray(Size size, double sigma){
	cvtColor(colored, grayscale, CV_RGB2GRAY);
	GaussianBlur(grayscale, grayscale, size,sigma, sigma);
}

/* ObjectModule: Light Detection Module*/

ObjectModule::ObjectModule(std::string name):
					VisionModule(name)
{
}

void ObjectModule::doVision(){}
void ObjectModule::present(){}


