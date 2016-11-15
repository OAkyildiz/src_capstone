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
template<class Target>
namespace debugUI {

//todo: add generic value type.
Element<Target>::Element(std::string title, std::string window,
		Target* target,int value, void (*)(int)) {
}

void Element<>::onChange() {
}


/* Trackbar */
template<class Target>
Trackbar<Target>::Trackbar(std::string title, std::string window, Target* target,
		int* param, int max, void (*)(int)):
		Element<Target>(title,window,target){
	createTrackbar(title,window,param,max,Trackbar::onChange,this );

}



// void Trackbar::onTrackbar(int val, void* ptr) {
// 	Trackbar* bar = (LightModule*)(ptr);
// 	mod->setThreshold(val);
// }

/* Radio Button */

RadioButton::RadioButton() {
}

// void RadioButton::onToggle(int val, void* ptr) {
// }

} /* namespace debugUI */
