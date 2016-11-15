/*
 * debugUI.h
 *
 *  Created on: Nov 13, 2016
 *      Author: oakyildiz, Ozan Akyıldız
 *		 Email: oakyildiz@wpi.edu
 *
 */

#ifndef SRC_VISION_DEBUGUI_H_
#define SRC_VISION_DEBUGUI_H_

template<typename Target>
namespace debugUI {

class Element{
public:
	Element(std::string title,std::string window,Target *target,int value,void(*)(int));
	virtual ~Element(){}
protected:
	Target* target_obj;
	Data value;
	void onChange();
};

class Trackbar: public Element{
public:
	Trackbar(std::string title,std::string window,Target *target,int *param,int max, void(*)(int));
	virtual ~Trackbar(){}

private:

	//static void onTrackbar(int val, void* ptr);


};

class RadioButton: public Element{
public:
	RadioButton(std::string title,std::string window,Target *target,int *param,int max, void(*)(int));
	virtual ~RadioButton(){}
private:
	//static void onToggle(int val, void* ptr);

};
} /* namespace debugUI */





#endif /* SRC_VISION_DEBUGUI_H_ */
