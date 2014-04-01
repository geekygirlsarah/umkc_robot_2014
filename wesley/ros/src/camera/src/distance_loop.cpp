#include <cv.h>
#include <highgui.h>

using namespace cv;

#include <iostream>
#include <sstream>
#include <cmath>

#include "tally_box.h"

#include <ros/ros.h>
#include <wesley/arm_point.h>


wesley::arm_point t;
bool waiting = true;
void arm_get_point(const wesley::arm_point& msg) {
	t.direct_mode = msg.direct_mode;
	t.x = msg.x;
	t.y = msg.y;
	t.z = msg.z;
	t.p = msg.p;
	t.r = msg.r;
	t.cmd = msg.cmd;

	waiting = false;
}

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

	double inital_area_px[4] = {
		0.0f,
		42452.6,
		23404.5,
		38908.9,
	};

	double inital_measure = 152.4;	// mm , ~6inches.

	// after some experimentation, my focal length constant was found to be:
	//
	//    f = 12281.13061 px^2 / mm
	//
	// please consult bound graph-notebook for more detailed calculations.

	return(sqrt(inital_area_px[tool] / apparent_area_px) * inital_measure);
//	return((12281.13061 * actual_area_of[tool]) / apparent_area);
}

struct tally {
	double average;
	unsigned int width;
};

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "distance");

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

	VideoCapture capture(0);
	if (!capture.isOpened()) {
		std::cerr << "unable to open default camera(0)" << std::endl;
		return(40);
	}

	const double tool_d[4] = {
		0.0f,
		72.4,
		57.15,
		50.8,
	};

	Mat frame, thresh;
	namedWindow("frame", CV_WINDOW_AUTOSIZE);

	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<wesley::arm_point>("/arm/put/point", 1000);
	ros::Subscriber sub = nh.subscribe("/arm/response", 1000, &arm_get_point);

	ROS_INFO("DIST --> waiting for /arm/response");
	do {
		// wait until the arm is in position to take a picture;
		ros::spinOnce();
	} while(waiting);

	std::vector<std::vector<Point> > contours;
	Moments mu;
	Point2f mc;
	const unsigned char frame_offset_y = 0;
	Point center(640 / 2, (480 - frame_offset_y) / 2);
//	float alpha = (atan2(t.y, t.x) - (3.14159 / 2));
	float alpha = (atan2(t.y, t.x));
//	Point3_<float> camera(t.x - 47, t.y - 27, t.z - 31.2);
	wesley::arm_point camera;
	camera.x = (t.x + (27*(cos(alpha)) + 47*(sin(alpha))));
	camera.y = (t.y - (47*(cos(alpha)) - 27*(sin(alpha))));
	camera.z = (t.z + (30));
	Point2f offset(0, 0);

	tally_box tally;

capture:
	for (int i = 7; i > 0; i--) {
		capture >> frame;
	}
	Rect ROI = Rect(0, frame_offset_y, 640, (480 - frame_offset_y));
	Mat viewport = frame(ROI);
	viewport.copyTo(thresh);
	cvtColor(thresh, thresh, CV_RGB2GRAY);
	threshold(thresh, thresh, 80, 255, CV_THRESH_BINARY);
	bitwise_xor(thresh, Scalar(255), thresh);

	// find initial contours
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
		// find the area of the approximated contour
		max_area = contourArea(approx);
		tally.append(max_area);

		// display the area on the frame
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

		// calculate the distance to the object along the camera's z axis
		double z_dist = distance(max_area, tool);
		// - and display
		ss << "dist: " << z_dist;
		putText(frame,
				ss.str(),
				Point(20, 60),
				FONT_HERSHEY_PLAIN,
				1.5,
				CV_RGB(0xFF, 0xDF, 0x00),
				2);
		ss.str("");

		// use moments to find the center of the contour
		mu = moments(approx, false);
		mc = Point2f((mu.m10 / mu.m00),
					 (mu.m01 / mu.m00));
		// - display the center
		ss << "cntr: " << mc;
		putText(frame,
				ss.str(),
				Point(300, 30),
				FONT_HERSHEY_PLAIN,
				1.5,
				CV_RGB(0xFF, 0xDF, 0x00),
				2);
		ss.str("");

		// offset of the contour center from the center of the camera's frame
		//    offset is currently in pixels
		offset.x = mc.x - center.x;
//		mc.y -= frame_offset_y;
		offset.y = -(mc.y - ((480 - frame_offset_y) / 2));
		// - and display the offset
		ss << "off_c: " << offset;
		putText(frame,
				ss.str(),
				Point(300, 60),
				FONT_HERSHEY_PLAIN,
				1.5,
				CV_RGB(0xFF, 0xDF, 0x00),
				2);
		ss.str("");

		// find the longest edge of the square tool (it's a rectangle)
		float off_x, off_y;
		if (( sqrt(pow((approx[0].x - approx[1].x), 2) + pow((approx[0].y - approx[1].y), 2)) ) >
		    ( sqrt(pow((approx[1].x - approx[2].x), 2) + pow((approx[1].y - approx[2].y), 2)) ) ) {
			off_x = approx[1].x - approx[0].x;
			off_y = approx[1].y - approx[0].y;
			putText(viewport, "ha", approx[1], FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0xEE, 0xF4, 0xF4), 1);
		} else {
			off_x = approx[2].x - approx[1].x;
			off_y = approx[2].y - approx[1].y;
			putText(viewport, "ha", approx[2], FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0xEE, 0xF4, 0xF4), 1);
		}
		// determine the offset angle of this edge from a straight up and down.
		//    this will be used to line the grip to the edge of the tool
		float off_r = atan2(off_y, off_x);
		float off_d = 90 - (off_r * 180 / 3.14159);
		// - and display
		ss << "off_d: " << off_d;
		putText(frame,
				ss.str(),
				Point(20, 120),
				FONT_HERSHEY_PLAIN,
				1.5,
				CV_RGB(0xFF, 0xDF, 0x00),
				2);
		ss.str("");

		double alpha_r = atan2(t.y, t.x);
//		theta = (theta * 180 / 3.14159);
		double alpha = (alpha_r * 180 / 3.14159) - 90;
		ss << "alpha: " << alpha;
		putText(frame,
				ss.str(),
				Point(20, 90),
				FONT_HERSHEY_PLAIN,
				1.5,
				CV_RGB(0xFF, 0xDF, 0x00),
				2);
		ss.str("");


//			ss << "h_dist: " << sqrt(pow((center.x - mc.x), 2) + pow((center.y - mc.y), 2));
		double hypot = sqrt(pow((approx[0].x - approx[2].x), 2) + pow((approx[0].y - approx[2].y), 2));
		ss << "hypot: " << hypot;
		putText(frame,
				ss.str(),
				Point(410, 190),
				FONT_HERSHEY_PLAIN,
				1.5,
				CV_RGB(0xFF, 0xDF, 0x00),
				2);
		ss.str("");

		// determine the ratio of mm / px to find the linear distance of the offset.
		const double ratio = tool_d[tool] / hypot;
		ss << "ratio: " << ratio;
		putText(frame,
				ss.str(),
				Point(410, 220),
				FONT_HERSHEY_PLAIN,
				1.5,
				CV_RGB(0xFF, 0xDF, 0x00),
				2);
		ss.str("");

		// translate the offset of contour center and frame center into mm.
		offset.x *= ratio;
		offset.y *= ratio;
		// angle of p from c along c's x-axis. this is in the camera's refernce frame.
		// the addition of pi comes from the angle of the camera in relation to the
		//    arm (wrist_roll);
		double theta_r = (atan2(offset.y, offset.x) + (3.14159 / 2));
		float xc = offset.x;
		float yc = offset.y;
		offset.x = ((xc * cos(theta_r)) + (yc * sin(theta_r)));
		// flipped from camera's negative y to robot's positive y.
		offset.y = -((yc * cos(theta_r)) - (xc * sin(theta_r)));
//		offset.x += 10 + (20 * cos((off_d * 3.14159 / 180.0)));
//		offset.y += 27 + (20 * sin((off_d * 3.14159 / 180.0)));

		ss << "off_m: " << offset;
		putText(frame,
				ss.str(),
				Point(300, 90),
				FONT_HERSHEY_PLAIN,
				1.5,
				CV_RGB(0xFF, 0xDF, 0x00),
				2);
		ss.str("");

		ss << "c: [" << camera.x
		   << ", " << camera.y
		   << ", " << camera.z
//		   << ", " << camera.p
//		   << ", " << camera.r
		   << "]";
		putText(frame,
				ss.str(),
				Point(20, 480),
				FONT_HERSHEY_PLAIN,
				1.5,
				CV_RGB(0xFF, 0xDF, 0x00),
				2);
		ss.str("");

		ss << "t: [" << t.x
		   << ", " << t.y
		   << ", " << t.z
		   << ", " << t.p
		   << ", " << t.r
		   << "]";
		putText(frame,
				ss.str(),
				Point(20, 450),
				FONT_HERSHEY_PLAIN,
				1.5,
				CV_RGB(0xFF, 0xDF, 0x00),
				2);
		ss.str("");

		wesley::arm_point p;
		p.direct_mode = true;
		p.x = t.x + offset.x;
		p.y = t.y + offset.y;
		p.z = t.z - z_dist - 25;
		p.p = t.p;
		p.r = t.r - off_d;
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

		polylines(viewport, approx, true, CV_RGB(0xD4, 0x35, 0xCD));
	//	drawContours(viewport, contours, biggest_contour, CV_RGB(0xD4, 0x35, 0xCD));
	//	drawContours(frame, approx, -1, CV_RGB(0xD4, 0x35, 0xCD));

		circle(viewport, mc, 5, CV_RGB(0x84, 0x43, 0xD6), CV_FILLED);
		circle(viewport, center, 2, CV_RGB(0x84, 0x43, 0xD6), CV_FILLED);
		circle(viewport, approx[0], 2, CV_RGB(0xE4, 0x83, 0x36), CV_FILLED);
		putText(viewport, "0", approx[0], FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0xEE, 0xF4, 0xF4), 1);
		circle(viewport, approx[1], 2, CV_RGB(0xE4, 0x83, 0x36), CV_FILLED);
		circle(viewport, approx[2], 2, CV_RGB(0x24, 0x43, 0xD6), CV_FILLED);
	}
	imshow("frame", frame);
	short keypress;
	do {
		keypress = waitKey();
		if(keypress == 27) {
			break;
		} else if (keypress == 105) {
			std::cout << approx << std::endl;
		} else if (keypress == 100) {
			std::cout << "approx.size(" << approx.size() << ")" << std::endl;
		} else if (keypress == 99) {
			goto capture;
		}
	} while(keypress != 27);

	std::cout << "areas: mean[" << tally.mean() << "] and mode[" << tally.mode() << "]" << std::endl;
	return(0);
}
