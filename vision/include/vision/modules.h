/*
 * modules.h
 *
 *  Created on: Nov 1, 2016
 *      Author: oakyildiz
 */

#ifndef INCLUDE_VISION_MODULES_H_
#define INCLUDE_VISION_MODULES_H_

#include "opencv2/opencv.hpp"
#include <vector>


extern int THRESHOLD;

using namespace cv;
namespace vision{

enum Detection{
	NONE=0,
	RED=10,
	GREEN=20,
	BLUE=30,
	OUTPUT=40,
	DETECTED=50

};

enum Type{
	RED_LED = 0,
	GREEN_LED = 1,
	BLUE_LED = 2,
	BUTTON = 3,
	DOOR_OPEN = 4,
	DOOR_CLOSED = 5
};

struct Camera{
	//Camera(bool, uint32_t, uint32_t, double, double, double, double, double, double, double);
	bool read;
	uint32_t height, width;
	double fx,fy;
	double cx_l,cx_r,cy;
	double Tx_l,Tx_r;
	string frame_id;

};

struct LEDColor{
	Mat* mask_ptr;
	Scalar value;
	std::string name;

};
const Scalar HSV_RED_LOW = Scalar(0,100,100);
const Scalar HSV_RED_HIGH = Scalar(3,255,255);

const Scalar HSV_RED_2_LOW = Scalar(177,100,100);
const Scalar HSV_RED_2_HIGH = Scalar(179,255,255);

const Scalar HSV_GREEN_LOW = Scalar(57,100,100);
const Scalar HSV_GREEN_HIGH = Scalar(63,255,255);

const Scalar HSV_BLUE_LOW = Scalar(118,100,100);
const Scalar HSV_BLUE_HIGH = Scalar(123,255,255);


class VisionModule{
public:
	VisionModule(std::string name);
	virtual ~VisionModule();

//	void setParent(VisionNode* node){
//		this->parent = node;
//	}
	//protected:
	virtual void doVision() = 0;
	void show();
	virtual void draw() = 0;
	virtual void print() = 0;
	Mat input;
	Mat input_R;
	Mat* output;
	Detection sel;

	vector<Mat*> framelist;
	void loadFrame(Mat img_in);
	void loadFrame_Stereo(Mat img_in);
	void loadCamera(Camera device);
	bool camIsSet();

	void setOutput(Mat *plug);
	void toggleOutput();

	const Camera& getCam() const {
	     return cam;
	       }

	const Point3d& getLocation() const {
	     return location;
	       }

	short int getType() const {
	     return type;
	       }

	Point getPxError() const {
	     return px_error;
	       }

protected:
	//VisionNode* parent;

	Camera cam;
	const std::string window_name;

	Mat	hsv;
	Mat red_mask;

	Point3d location;
    Type type;
	vector<vector<Point> > contours_poly;
	Point centroid, centroid_R;
	Point px_error;

	Point3d calculateLocation(Point L, Point R);
	Point findVisualPair(int color_index);
	vector<Mat> seperateChannels(Mat in);
	Point getPixDistFromCenter(Point p1,Point p2);

	Point getCentroid(Mat in);
	void getRectangle(Mat in);
	int checkSingleColor(Mat in, bool withRectangle);

	static void onTrackbar(int val, void* ptr);
	static void onTrackbar2(int val, void* ptr);
	static void onToggle(int state, void* ptr);
	static void mouseHandler(int event, int x, int y, int flags, void* param);

	void autoCloseWindow(){

		//FIX
		if (getWindowProperty(window_name,4)==0){
			exit(0);
		}
	}
};

class LightModule: public VisionModule{

public:
	LightModule(std::string name,short int t);
	virtual ~LightModule(){}

	void doVision();
	void draw();
	void print();

	void setThreshold(short int threshold) {
		this->threshold = threshold;
	}

	void setBlurSize(short int size) {
		this->blursize = size;
	}
	void setBlur(short int state) {
		//this->blur = state;
	}
	short int getThreshold() const {
		return threshold;
	}

private:
	/* parameters*/
	short int threshold, blursize;
	int slider,slider2;

	/* members */

	Mat green_mask;
	Mat blue_mask;
	Mat* active_mask;

	Mat	hue;
	Mat	saturation;
	Mat	value;

	Mat	extra1;

	Mat grayscale;

	/* outputs */
	//TODO:: Structurize the output data
	//vector<vector<Point> > *contours_poly;
	Scalar color;
	std::string color_text;

	/*vision methods*/
	void LEDDetection();
	void colorFromMaxIntensity();

	/*image helpers*/
	void cvtGray();
	void blurInput();
	Mat getSingleLayer(Mat in, int layer);
	void getHSVChannels(Mat in);
	Scalar setTargetColor(Mat in, Point p);
	/* Interaction handlers*/
	static void onTrackbar(int val, void* ptr);
	static void onTrackbar2(int val, void* ptr);
	static void onToggle(int state, void* ptr);

};

class ObjectModule: public VisionModule{
public:
	ObjectModule(std::string name);
	virtual ~ObjectModule(){}

	void doVision();
	void draw();
	void print();
private:
	/* parameters */
	/* members */
	void onTrackbar(int val, void* ptr){}
	void onTrackbar2(int val, void* ptr){}
	void onToggle(int state, void* ptr){}
};
}



#endif /* INCLUDE_VISION_MODULES_H_ */
