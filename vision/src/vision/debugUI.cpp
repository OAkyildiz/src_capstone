/*
 * debugUI.cpp
 *
 *  Created on: Nov 13, 2016
 *      Author: oakyildiz, Ozan Akyıldız
 *		 Email: oakyildiz@wpi.edu
 *
 */

#include "vision/debugUI.h"

#include "opencv2/opencv.hpp"

using namespace cv;
namespace debugUI {

template<class Target, class Data>
Element<Target,Data>::Element(std::string title, std::string window,
		Target* target,Data value, void (*)(int)) {
}

void Element::onChange() {
}

template<class Target>
Trackbar<Target>::Trackbar(std::string title, std::string window, Target* target,
		int* param, int max, void (*)(int)):
		Element(title,window,target){
	createTrackbar(title,window,param,max,Trackbar::onTrackbar,this );

}

/* Trackbar */


void Trackbar::onTrackbar(int val, void* ptr) {
	Trackbar* bar = (LightModule*)(ptr);
	mod->setThreshold(val);
}
/* Radio Button */
RadioButton::RadioButton() {
}


void RadioButton::onToggle(int val, void* ptr) {
}

} /* namespace debugUI */

