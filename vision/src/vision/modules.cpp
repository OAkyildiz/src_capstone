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
VisionModule::VisionModule(std::string name):
		framelist(0)
{
	namedWindow(window_name, 1);
	startWindowThread();
	int mouseParam= CV_EVENT_FLAG_LBUTTON;
	setMouseCallback(window_name,VisionModule::mouseHandler,this);

	setOutput(&input);
	framelist.push_back(&input);
}

VisionModule::~VisionModule(){
	destroyWindow(window_name);
}

void VisionModule::getFrame(Mat img_in) {

	input = img_in;
}
void VisionModule::setOutput(Mat *plug) {
	output=plug;
}
void VisionModule::toggleOutput(){
	vector<Mat*>::iterator it,end;
	int next;
	//end = framelist.end();
    it=find(framelist.begin(),framelist.end(),output);
	 if(it != framelist.end()){
		 auto int pos = it - framelist.begin()+1;
		 ROS_INFO("current: %d", int(pos));
		 if(pos==framelist.size()){
			 next=0;
			 setOutput(framelist[next]);
		 }
		 else
			 next=pos;
			 setOutput(framelist[next]);
		 ROS_INFO("switching output to %d of %d",next+1,int(framelist.size()));
	 }




}
void VisionModule::mouseHandler(int event, int x, int y, int flags, void* ptr)
{
	VisionModule* mod = (VisionModule*)(ptr);
	Mat hsv_im;
	Scalar rgb, hsv;
	Point pix;
	Vec3b pixel;
	    switch(event){
	    case CV_EVENT_RBUTTONDOWN:
	    	    	mod->toggleOutput();
	    	    	break;
	    case CV_EVENT_LBUTTONDOWN:
	        ROS_INFO("**Mouse clicked**");
	        break;
	    case CV_EVENT_LBUTTONUP:
	    	pix = Point(x,y);
        	pixel = mod->input.at<Vec3b>(pix);
        	rgb = Scalar(pixel.val[0], pixel.val[1], pixel.val[2]);

        	cvtColor(mod->input, hsv_im, CV_BGR2HSV);
        	pixel = hsv_im.at<Vec3b>(pix);
        	hsv = Scalar(pixel.val[0], pixel.val[1], pixel.val[2]);

        	ROS_INFO("At (%d,%d): RGB %.0f,%.0f,%.0f", pix.x,pix.y,rgb[0], rgb[1], rgb[2]);
        	ROS_INFO(" HSV: %.0f,%.0f,%.0f \n", hsv[0], hsv[1], hsv[2]);
        	break;

	    }
	}
///////////
/* LightModule: Light Detection Module*/

LightModule::LightModule(std::string name, short int t):
					VisionModule(name),
					threshold(t),slider(t),
					blursize(3), slider2(blursize),
					button(1),
					max(0), r(0),
					color(0,0,0)
{
	framelist.push_back(&grayscale);

	framelist.push_back(&hue);
	framelist.push_back(&saturation);
	framelist.push_back(&value);

	framelist.push_back(&red_mask);
	framelist.push_back(&blue_mask);
	framelist.push_back(&green_mask);


	createTrackbar( "threshold", window_name, &slider,255,LightModule::onTrackbar, this);
	createTrackbar( "blur size", window_name, &slider2,50,LightModule::onTrackbar2, this);

	//TODO: Build opencv with QT support
	//createButton("blur", LightModule::onToggle, &button, CV_RADIOBOX, button);

}

void LightModule::doVision(){
	//inRange(input, cv::Scalar(0, 0, 0), cv::Scalar(180, 180, 180), colored);
	if (blursize) blurInput();
	cvtGray();
	getHSVLayers(input);
	seperateChannels(input,hsv);

	circle(*output, getCentroid(red_mask), 10,  Scalar(179,200,200), 3, 8, 0);

	//print();
}

void LightModule::show(){
	imshow(window_name, *output);
}
void LightModule::print(){
    //waitKey(5);

	// debug
	// color
	ROS_INFO("val: %.0f  at (%d, %d)	",max, max_pt.x, max_pt.y);
	ROS_INFO("r: %d    BGR:(%.0f, %.0f, %.0f) \n",r/2 , color[0], color[1], color[2] );
	// exit on key
	//if(waitKey(30) >= 0) break;
}

/* color operations*/
void LightModule::colorFromMaxIntensity() {
	//cvtGray(blursize, 3);
		//cv::threshold(grayscale, grayscale,slider,0,THRESH_TOZERO_INV);
		//seperateChannels(input,output);

		double min;
		minMaxLoc(grayscale, &min, &max, &min_pt, &max_pt);

		if (max > 130){
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
}


void LightModule::seperateChannels(Mat in, Mat out){
	// create local Mats

	Mat red1,red2;
	Mat mask_med,mask;

	//red
	inRange(hsv, HSV_RED_LOW,HSV_RED_HIGH, red1);
	inRange(hsv, HSV_RED_2_LOW,HSV_RED_2_HIGH, red2);
	bitwise_or(red1,red2,red_mask);

	//blue
	inRange(hsv,HSV_BLUE_LOW, HSV_BLUE_HIGH, blue_mask);

	//green
	inRange(hsv, HSV_GREEN_LOW,HSV_GREEN_HIGH, green_mask);


	//finalize
	//bitwise_or(red_mask,blue_mask, mask_med);
	//bitwise_or(mask_med, green_mask, mask);
	//bitwise_and(hsv_img,mask,out);
}
Point LightModule::getCentroid(Mat in){
	Moments m = moments((in>=50),true);
	Point2d p(m.m10/m.m00, m.m01/m.m00);
	return p;
}

/*image helpers*/
void LightModule::cvtGray(){
	cvtColor(input, grayscale, CV_BGR2GRAY);

}
void LightModule::blurInput(){
	int s=blursize*2+1;
	GaussianBlur(input, input, Size(s,s),3, 3);
}

Mat LightModule::getSingleLayer(Mat in, int layer){
	vector<Mat> out;
	split(in,out);
	return out[layer];
}
void LightModule::getHSVLayers(Mat in){
	vector<Mat> out;
	cvtColor(in, hsv, CV_BGR2HSV);
	split(hsv,out);
	hue = out[0];
	saturation = out[1];
	value = out[2];
}

/* Interaction handlers*/
void LightModule::onTrackbar(int val, void* ptr){
	LightModule* mod = (LightModule*)(ptr);
	mod->setThreshold(val);
}
void LightModule::onTrackbar2(int val, void* ptr){
	if (val)ROS_INFO("blurring with %d", val*2+1);
	else ROS_INFO("no blur");
	LightModule* mod = (LightModule*)(ptr);
	mod->setBlurSize(val);
}




void LightModule::onToggle(int state, void* ptr){
	LightModule* mod = (LightModule*)(ptr);
	mod->setBlur(state);

}

//////////////
/* ObjectMoudel: Object recognition module*/
ObjectModule::ObjectModule(std::string name):
					VisionModule(name)
{
}

void ObjectModule::doVision(){}
void ObjectModule::show(){}
void ObjectModule::print(){}
