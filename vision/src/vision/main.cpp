/*
 * main.cpp
 *
 *  Created on: Sep 20, 2016
 *      Author: oakyildiz
 */

#include <stdio.h>
#include "opencv2/opencv.hpp"

using namespace cv;

int main(int, char**)
{
    cv::VideoCapture cap(0); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
        return -1;

    Mat grayscale;
    namedWindow("webcam",1);
    for(;;)
    {
        Mat frame;
        cap >> frame; // get a new frame from camera

        cvtColor(frame, grayscale, CV_RGB2GRAY);
        GaussianBlur(grayscale, grayscale, Size(7,7), 1.5, 1.5);

        double min, max;
        int r = 0, treshold = 130;

        cv::Point  min_pt, max_pt;
        cv::minMaxLoc(grayscale, &min, &max, &min_pt, &max_pt);

        //Canny(edges, edges, 0, 30, 3);
        Scalar color;
        if (max > treshold){
            // dynamic radius
        	r = max / 10 + 2;
        	Vec3b pixel = frame.at<Vec3b>(max_pt);
        	color = Scalar(pixel.val[0]-r, pixel.val[1]-r, pixel.val[2]-r);
        	// constant radius
        	//int r=10;


        	circle(frame, max_pt, r,  color, 3, 8, 0);
        }
        else{
        	color = Scalar(0,0,0);
        	r = 0;

        }

        imshow("webcam", frame);

        // debug
        // color
        printf("val: %.0f  at (%d, %d)	",max, max_pt.x, max_pt.y);
        printf("r: %d    BGR:(%.0f, %.0f, %.0f) \n",r/2 , color[0], color[1], color[2] );
        // exit on key
        if(waitKey(30) >= 0) break;
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}


