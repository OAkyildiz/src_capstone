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
/* VisionMoudel: Base class*/
VisionModule::VisionModule(std::string name):
				framelist(0),
				cam(),
				sel(NOLED)

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

void VisionModule::loadFrame(Mat img_in) {

	input = img_in;
}

void VisionModule::loadFrame_Stereo(Mat img_in) {

	input_R = img_in;
}

void VisionModule::loadCamera(Camera device){
	cam = device;
	cam.read = true;
}

bool VisionModule::camIsSet(){
	return cam.read;
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
		//ROS_INFO("current: %d", int(pos));
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
							blursize(1), slider2(blursize),
							color(0,0,0),color_text("")
{

	active_mask=NULL;
	contours_poly= vector<vector<Point> >(1);

//	framelist.push_back(&grayscale);

//	framelist.push_back(&hue);
//	framelist.push_back(&saturation);
//	framelist.push_back(&value);

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
	cvtColor(input, hsv, CV_BGR2HSV);

	vector<Mat> masks=seperateChannels(hsv);
	red_mask = masks[0];
	green_mask = masks[1];
	blue_mask = masks[2];
	LEDDetection();

}

void LightModule::doDisparity(int color_index){
	Mat hsv_R;
	cvtColor(input_R, hsv_R, CV_BGR2HSV);
	vector<Mat> masks = seperateChannels(hsv_R);
	Point centr_R= getCentroid(masks[color_index]);
	ROS_INFO("[RIGHT]	 at %d,%d", centr_R.x, centr_R.y);

}

void LightModule::show(){
	imshow(window_name, *output);
	//if(waitKey(30) >= 0) exit(0);
}
void LightModule::draw(){
	//waitKey(5);
	circle(*output, centroid, 2,  color, 1, 8, 0);
	drawContours( *output, contours_poly, 0, color, 1, 8, vector<Vec4i>(), 0, Point() );

	//add(LED,*output,*output);
}
void LightModule::print(){
	ROS_INFO("[LEFT] %s at %d,%d", color_text.data(), centroid.x, centroid.y);

}
/* color operations*/
void LightModule::LEDDetection(){
	int index;
	switch (sel) {
	default:
	case NOLED:
		sel=DETECTED;
		if 	(checkSingleLed(red_mask)){
			active_mask=&red_mask;
			color_text="RED";
			index=0;
				}
		else if (checkSingleLed(green_mask)){
			active_mask=&green_mask;
			color_text="GREEN";
			index=1;

				}
		else if	(checkSingleLed(blue_mask)){
			active_mask=&blue_mask;
			color_text="BLUE";
			index=2;

		}
		else{
			sel=NOLED;
			break;
		}
		break;

	case OUTPUT:
		//seper
		print();
		doDisparity(index);
		draw();

		break;


	case DETECTED:

		if(countNonZero(*active_mask))
			draw();
		else{
				active_mask = NULL;
				centroid = Point(0,0);
				color = Scalar(0,0,0);
				sel = NOLED;
		}

		break;
	}
}
void LightModule::colorFromMaxIntensity() {

	int r;
	double min,max;
	Point min_pt, max_pt;
	minMaxLoc(grayscale, &min, &max, &min_pt, &max_pt);

	if (max > 130){
		// dynamic radius
		r = max / 10 + 2;
		Vec3b pixel = input.at<Vec3b>(max_pt);
		color = Scalar(pixel.val[0]-r, pixel.val[1]-r, pixel.val[2]-r);
		// constant radius
		// int r=10;
		circle(grayscale, max_pt, 5,  color, 1, 8, 0);
	}
	else{
		color = Scalar(0,0,0);
		r = 0;

	}
}
int LightModule::checkSingleLed(Mat in){
	int count = countNonZero(in);
	if(count>35) {
		//ROS_INFO("Blue pix count: %d", count);//ROS_INFO("pix count: %d", cr);
		centroid = getCentroid(in);
		getRectangle(in);
	}
	else count=0;
	return count;

}

vector<Mat> LightModule::seperateChannels(Mat in){
	// create local Mats

	Mat red1,red2;
	Mat mask_med,mask;
	vector<Mat> out(3);

	//red
	inRange(in, HSV_RED_LOW,HSV_RED_HIGH, red1);
	inRange(in, HSV_RED_2_LOW,HSV_RED_2_HIGH, red2);
	bitwise_or(red1,red2,out[0]);

	//blue
	inRange(in,HSV_BLUE_LOW, HSV_BLUE_HIGH,out[1]);

	//green
	inRange(in, HSV_GREEN_LOW,HSV_GREEN_HIGH, out[2]);

	return out;

}
Point LightModule::getCentroid(Mat in){
	Moments m = moments((in>=50),true);
	Point2d p(m.m10/m.m00, m.m01/m.m00);
	return p;
}
void LightModule::getRectangle(Mat in){
	/// Function header


	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	// Approximate contours to polygons + get bounding rects and circles
	findContours( in, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

	//vector<Rect> boundRect( contours.size() );
	//vector<Point2f>center( contours.size() );
	//vector<float>radius( contours.size() );

	approxPolyDP( Mat(contours[0]), contours_poly[0], 3, true );
		//boundRect[i] = boundingRect( Mat(contours_poly[i]) );




	/// Draw polygonal contour + bonding rects + circles


		//rectangle( *output, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );


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


//void LightModule::getHSVChannels(Mat in){
//	vector<Mat> channnels;
//	cvtColor(in, hsv, CV_BGR2HSV);
//	split(hsv,channels);
//	hue = channels[0];
//	saturation = channels[1];
//	value = channels[2];
//}
Scalar LightModule::setTargetColor(Mat in, Point p){
	Vec3b pixel = input.at<Vec3b>(p);
	return Scalar(pixel.val[0], pixel.val[1], pixel.val[2]);
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
void ObjectModule::draw(){}
void ObjectModule::print(){}
