/** THIS CODE IS OBSOLETED BY ID_TOOL.CPP
 * 
 *  distance.cpp
 *  by: Eric M Gonzalez
 * 
 *  PURPOSE: This code was used to narrow in on an algorithm to find
 *           the top and distance of a tool in the id_tool program.
 *
 *           Through the use of this program, and the help of Darren
 *           Cogan, id_tool.cpp was written to pick up the tool.
 * 
 *           This program will still work, but will give values that
 *           do not match the robot's reference frame or the values
 *           returned by id_tool.
 * 
 *           Call this program with one of {t, c, s} to search for
 *           't'riangle, 'c'ircle, or 's'quare.
 *           
 */



#include <cv.h>
#include <highgui.h>

using namespace cv;

#include <iostream>
#include <sstream>
#include <cmath>

#include "tally_box.h"

#include <ros/ros.h>
#include <wesley/arm_point.h>

double distance(double apparent_area_px, short tool=0) {
	// formula for this is a linear porpotionality:
	// 
	// focal length is the ratio of pixels to centimeters for camera
	//    focal length of camera:		f
	//    known distance to object:		Z	mm
	//    known size of object at 'd':	S	mm^2
	//    size of object in pixels:		P	px^2
	//
	//    f = Z * (S / P)
	//
	// solving for Z:
	//
	//    Z = (P / S) * f     -->   Z = (P' / S) * f
	//                              Z = P' * (f / S)
	//
	// this function takes two arguments: the apparent size in square pixels
	//    of the object as found by contourArea(), and the particular shape
	//    that is being sought. it then returns an estimate of the distance
	//    to the object in the Z-plane of the camera.
	//
	// it is up to the caller to do something with this information!

//	double actual_area_of[4] = {
//		161.29,
//		2450.3175,		// square
//		1414.2722529,	// trianlge
//		2026.829916,	// circle
//	};
	// after some experimentation, my focal length constant was found to be:
	//
	//    f = 12281.13061 px^2 / mm
	//
	// please consult bound graph-notebook for more detailed calculations.
//	return((12281.13061 * actual_area_of[tool]) / apparent_area);
	


	// The above commentary is now obsoleted. Thanks again to Darren for the
	//    following method of determining distance. please see id_tool.cpp
	//    for more information.
	double inital_area_px[4] = {
		0.0f,
		42452.6,
		23404.5,
		38908.9,
	};

	double inital_measure = 152.4;	// mm , ~6inches.

	return(sqrt(inital_area_px[tool] / apparent_area_px) * inital_measure);
}

struct tally {
	double average;
	unsigned int width;
};

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "distance");

	// what are we looking for? see if we were called with an argument.
	enum tool_shapes { NONE, SQUARE, TRIANGLE, CIRCLE };
	short tool = NONE;
	if (argc == 2) {
		char given = argv[1][0];
		switch(given) {
			case '1': case 's':
				tool = SQUARE;
				break;
			case '2': case 't':
				tool = TRIANGLE;
				break;
			case '3': case 'c':
				tool = CIRCLE;
				break;
			default:
				tool = NONE;
				break;
		}
	}

//	VideoCapture capture(0);
//	if (!capture.isOpened()) {
//		std::cerr << "unable to open default camera(0)" << std::endl;
//		return(40);
//	}

	Mat frame, thresh;
	namedWindow("frame", CV_WINDOW_AUTOSIZE);
	frame = imread("snapshot.png");

	// create a publisher - however, this isn't used internally.
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<wesley::arm_point>("/arm/put/point", 1000);

	wesley::arm_point t;
	t.direct_mode = true;
	t.x = -28.8;
	t.y = 243.0;
	t.z = 149.0;
	t.p = -85.0;
	t.r =  90.0;
	t.cmd = "grip";

	std::vector<std::vector<Point> > contours;
	Moments mu;
	Point2f mc;
	Point center(320, 240);
	Point3_<float> camera(t.x - 47, t.y - 27, t.z - 31.2);
	Point2f offset(0, 0);

	tally_box tally;

	cvtColor(frame, thresh, CV_RGB2GRAY);
	threshold(thresh, thresh, 80, 255, CV_THRESH_BINARY);
	bitwise_xor(thresh, Scalar(255), thresh);

	findContours(thresh,
				 contours,
				 CV_RETR_EXTERNAL,
				 CV_CHAIN_APPROX_NONE);

	// find biggest area;
	double max_area = 0.0f;
	int biggest_contour = -1;
	for (int ath = 0; ath < contours.size(); ath++) {
		double tmp_area = contourArea(contours[ath]);
		if (tmp_area > max_area) {
			max_area = tmp_area;
			biggest_contour = ath;
		}
	}

	// use the largest contour found and run that through approxPolyDP.
	std::vector<Point> approx;
	if (biggest_contour != -1) {
		approxPolyDP(contours[biggest_contour],
					 approx,
					 arcLength(contours[biggest_contour], true) * .015,
					 true);
		max_area = contourArea(approx);
		tally.append(max_area);

		std::stringstream ss;
		ss << "area: " << max_area;
		putText(frame,
				ss.str(),
				Point(20, 30),
				FONT_HERSHEY_PLAIN,
				1.5,
				CV_RGB(0xFF, 0xDF, 0x00),
				2);
		ss.str("");
		double z_dist = distance(max_area, tool);
		ss << "dist: " << z_dist;
		putText(frame,
				ss.str(),
				Point(20, 60),
				FONT_HERSHEY_PLAIN,
				1.5,
				CV_RGB(0xFF, 0xDF, 0x00),
				2);
		ss.str("");

		mu = moments(approx, false);
		mc = Point2f((mu.m10 / mu.m00),
					 (mu.m01 / mu.m00));
		ss << "cntr: " << mc;
		putText(frame,
				ss.str(),
				Point(300, 30),
				FONT_HERSHEY_PLAIN,
				1.5,
				CV_RGB(0xFF, 0xDF, 0x00),
				2);
		ss.str("");

	//	mu = moments(contours[biggest_contour], false);
		offset.x = mc.x - camera.x;
		offset.y = mc.y - camera.y;

		ss << "offs: " << offset;
		putText(frame,
				ss.str(),
				Point(300, 60),
				FONT_HERSHEY_PLAIN,
				1.5,
				CV_RGB(0xFF, 0xDF, 0x00),
				2);
		ss.str("");

		float off_x, off_y;
		off_x = approx[3].x - approx[2].x;
		off_y = approx[3].y - approx[2].y;
		float off_r = atan2(off_y, off_x);
		float off_d = 90 - -(off_r * 180 / 3.14159);

		ss << "off_d: " << off_d;
		putText(frame,
				ss.str(),
				Point(20, 120),
				FONT_HERSHEY_PLAIN,
				1.5,
				CV_RGB(0xFF, 0xDF, 0x00),
				2);
		ss.str("");

		wesley::arm_point p;
		p.direct_mode = true;
		p.x = t.x + offset.x;
		p.y = t.y + offset.y;
		p.z = t.z - z_dist;
		p.p = t.p;
		p.r = t.r + off_d;
		p.cmd = "pick";

		ss << "p: [" << p.x
		   << ", " << p.y
		   << ", " << p.z
		   << ", " << p.p
		   << ", " << p.r
		   << "]";
		putText(frame,
				ss.str(),
				Point(20, 420),
				FONT_HERSHEY_PLAIN,
				1.5,
				CV_RGB(0xFF, 0xDF, 0x00),
				2);
		ss.str("");

		drawContours(frame, contours, biggest_contour, CV_RGB(0xD4, 0x35, 0xCD));
	//	drawContours(frame, approx, -1, CV_RGB(0xD4, 0x35, 0xCD));

		circle(frame, mc, 5, CV_RGB(0x84, 0x43, 0xD6), CV_FILLED);
	}
	imshow("frame", frame);
	short keypress = 0;
	do {
		keypress = waitKey();
		if(keypress == 27) {
			break;
		} else if (keypress == 105) {
			std::cout << approx << std::endl;
		} else if (keypress == 100) {
			std::cout << "approx.size(" << approx.size() << ")" << std::endl;
		}
	} while (keypress != 27);

	std::cout << "areas: mean[" << tally.mean() << "] and mode[" << tally.mode() << "]" << std::endl;
	return(0);
}
