/*
 * modules.h
 *
 *  Created on: Nov 1, 2016
 *      Author: oakyildiz
 */

#ifndef INCLUDE_VISION_MODULES_H_
#define INCLUDE_VISION_MODULES_H_

#include "opencv2/opencv.hpp"


extern int THRESHOLD;

using namespace cv;
namespace vision{

const Scalar HSV_RED_LOW = Scalar(0,100,100);
const Scalar HSV_RED_HIGH = Scalar(15,255,255);

const Scalar HSV_RED_2_LOW = Scalar(160,100,100);
const Scalar HSV_RED_2_HIGH = Scalar(179,255,255);

const Scalar HSV_GREEN_LOW = Scalar(20,100,100);
const Scalar HSV_GREEN_HIGH = Scalar(30,255,255);

const Scalar HSV_BLUE_LOW = Scalar(34,100,100);
const Scalar HSV_BLUE_HIGH = Scalar(80,255,255);


class VisionModule{
public:
	VisionModule(std::string name);
	virtual ~VisionModule();

//protected:
	virtual void doVision() = 0;
	virtual void present() = 0;

	Mat input;
	Mat output;


protected:
	std::string window_name;

//private:


};

class LightModule: public VisionModule{

public:
	LightModule(std::string name,short int t);
	virtual ~LightModule(){}

	void doVision();

	void setThreshold(short int threshold) {
		this->threshold = threshold;
	}

	short int getThreshold() const {
		return threshold;
	}

private:
	/* parameters*/
	short int threshold;

	/* members */
	double max;
	int r;
	Scalar color;

	Mat red_ch;
	Mat green_ch;
	Mat blue_ch;

	Mat grayscale;

	Point min_pt;
	Point max_pt;
	void cvtGray(Size size,double sigma);
	void seperateChannels(Mat in, Mat out);
	void present();
};

class ObjectModule: public VisionModule{
public:
	ObjectModule(std::string name);
	virtual ~ObjectModule(){}

	void present();
	void doVision();

private:
	/* parameters */

	/* members */

};
}



#endif /* INCLUDE_VISION_MODULES_H_ */
