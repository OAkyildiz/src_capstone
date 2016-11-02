/*
 * modules.h
 *
 *  Created on: Nov 1, 2016
 *      Author: oakyildiz
 */

#ifndef INCLUDE_VISION_MODULES_H_
#define INCLUDE_VISION_MODULES_H_

#include "opencv2/opencv.hpp"

using namespace cv;
namespace vision{


class VisionModule{
public:
	VisionModule(std::string name);
	virtual ~VisionModule();

//protected:
	virtual void doVision() = 0;
	virtual void present() = 0;

protected:
	Mat colored;
	std::string window_name;

//private:


};

class LightModule: public VisionModule{

public:
	LightModule(std::string name,short int t);
	virtual ~LightModule(){}

	void doVision();
private:
	/* parameters*/
	short int threshold;

	/* members */
	double max;
	int r;
	Scalar color;
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
