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
	//inRange(input, cv::Scalar(0, 0, 0), cv::Scalar(180, 180, 180), colored);
	cvtGray(Size(7,7), 2);
	seperateChannels(input,output);

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


		circle(grayscale, max_pt, r,  color, 3, 8, 0);
	}
	else{
		color = Scalar(0,0,0);
		r = 0;

	}
	present();
}
void LightModule::present(){
	imshow(window_name, output);
    //waitKey(5);

	// debug
	// color
	ROS_INFO("val: %.0f  at (%d, %d)	",max, max_pt.x, max_pt.y);
	ROS_INFO("r: %d    BGR:(%.0f, %.0f, %.0f) \n",r/2 , color[0], color[1], color[2] );
	// exit on key
	//if(waitKey(30) >= 0) break;
}

void LightModule::cvtGray(Size size, double sigma){
	cvtColor(input, grayscale, CV_RGB2GRAY);
	GaussianBlur(grayscale, grayscale, size,sigma, sigma);
}

void LightModule::seperateChannels(Mat in, Mat out){
	// create local Mats
	Mat hsv_img, red1,red2, interim;
	// convert colors
	cvtColor(in, hsv_img, cv::COLOR_BGR2HSV);

	//red
	inRange(hsv_img, HSV_RED_LOW,HSV_RED_HIGH, red1);
	inRange(hsv_img, HSV_RED_2_LOW,HSV_RED_2_HIGH, red2);
	add(red1,red2,red_ch);

	//blue
	inRange(hsv_img,HSV_BLUE_LOW, HSV_BLUE_HIGH, blue_ch);

	//green
	inRange(hsv_img, HSV_GREEN_LOW,HSV_GREEN_HIGH, green_ch);

	//finalize
	add(red_ch,blue_ch, interim);
	//add(interim, green_ch, hsv_img);
	cvtColor(hsv_img, out, cv::COLOR_HSV2BGR);


}

/* ObjectModule: Light Detection Module*/

ObjectModule::ObjectModule(std::string name):
					VisionModule(name)
{
}

void ObjectModule::doVision(){}
void ObjectModule::present(){}


