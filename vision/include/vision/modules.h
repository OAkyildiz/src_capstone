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


class VisionModule{
public:
	VisionModule(std::string name);
	virtual ~VisionModule();

//protected:
	virtual void doVision() = 0;
	virtual void present() = 0;

	Mat colored;

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

	Mat extra1;
	Mat extra2;
	Mat grayscale;

	Point min_pt;
	Point max_pt;
	void cvtGray(Size size,double sigma);
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
