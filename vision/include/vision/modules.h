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

//protected:
	virtual void doVision() = 0;
	virtual void show() = 0;
	virtual void print() = 0;

	Mat input;
	Mat* output;

	vector<Mat*> framelist;

	void getFrame(Mat img_in);
	void setOutput(Mat *plug);
	void toggleOutput();

protected:
	const std::string window_name;
	static void onTrackbar(int val, void* ptr);
	static void onTrackbar2(int val, void* ptr);
	static void onToggle(int state, void* ptr);
	static void mouseHandler(int event, int x, int y, int flags, void* param);

	//virtual static void onTrackbar( int, void* );

//private:


};

class LightModule: public VisionModule{

public:
	LightModule(std::string name,short int t);
	virtual ~LightModule(){}

	void doVision();
	void show();
	void print();

	void setThreshold(short int threshold) {
		this->threshold = threshold;
	}

	void setBlurSize(short int size) {
		this->blursize = size;
	}
	void setBlur(short int state) {
		this->blur = state;
	}
	short int getThreshold() const {
		return threshold;
	}

private:
	/* parameters*/
	short int threshold, blursize, blur;
	int slider,slider2, button;

	double max;
	int r;

	/* members */

	Scalar color;

	Mat red_mask;
	Mat green_mask;
	Mat blue_mask;

	Mat	hue;
	Mat	saturation;
	Mat	value;

	Mat	extra1;

	Mat	hsv;
	Mat grayscale;

	Point min_pt;
	Point max_pt;

	/*vision methods*/
	void colorFromMaxIntensity();
	void seperateChannels(Mat in, Mat out);
	Point getCentroid(Mat in);

	/*image helpers*/
	void cvtGray();
	void blurInput();
	void getRedMask();
	void getGreenMask();
	void getBlueMask();

	Mat getSingleLayer(Mat in, int layer);
	void getHSVLayers(Mat in);
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
	void show();
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
