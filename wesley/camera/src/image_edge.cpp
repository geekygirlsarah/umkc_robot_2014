#include <highgui.h>
#include <cv.h>
#include <iostream>
#include <string>
#include <vector>
#include <sstream>

using namespace std;
using namespace cv;

Mat frame;
Mat edges;
Mat grey;

int lo_thresh;
int hi_thresh;
int max_thresh = 1000;
int min_thresh = 0;

void find_shapes() {
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(edges, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));

	for(size_t ith = 0; ith < contours.size(); ith++) {
		vector<Point> approx;
		approxPolyDP(Mat(contours[ith]), approx, arcLength(Mat(contours[ith]), true)*.02, true);
		switch(approx.size()) {
			case 4: case 5:
				drawContours(frame, contours, ith, CV_RGB(222, 78, 0), -1);
				break;
			case 6:
				drawContours(frame, contours, ith, CV_RGB(55, 0, 200), -1);
				break;
			default:
			//	if (approx.size() > 6) {
			//		drawContours(frame, contours, ith, CV_RGB(103,227,0), -1);
			//	}
				break;
		}
	}

	std::vector<Vec3f> circles;
	HoughCircles(grey, circles, CV_HOUGH_GRADIENT, 2, grey.rows/4, 200, 100);

	for (size_t j = 0; j < circles.size(); j++) {
		int radius = cvRound(circles[j][2]);
		if (radius > 80) {
			Point center(cvRound(circles[j][0]), cvRound(circles[j][1]));
			circle( frame, center, 3, CV_RGB(200, 55, 0), -1, 8, 0);
			circle( frame, center, radius, Scalar(55, 200, 0), 3, 8, 0 );
		}
	}

	imshow("image raw", frame);
}


void slider_lo_thresh( int, void* ) {
	Canny(grey, edges, lo_thresh, hi_thresh);
	imshow("image edges", edges);
	find_shapes();
}

void slider_hi_thresh( int, void* ) {
	find_shapes();
}	

void find_triangle() {
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(edges, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));

	for(size_t ith = 0; ith < contours.size(); ith++) {
		vector<Point> approx;
		approxPolyDP(Mat(contours[ith]), approx, arcLength(Mat(contours[ith]), true)*.02, true);
		switch(approx.size()) {
			case 3:
				drawContours(frame, contours, ith, CV_RGB(222, 78, 0), -1);
				break;
			default:
			//	if (approx.size() > 6) {
			//		drawContours(frame, contours, ith, CV_RGB(103,227,0), -1);
			//	}
				break;
		}
	}
	imshow("image raw", frame);
}

void find_square() {
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(edges, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));

	for(size_t ith = 0; ith < contours.size(); ith++) {
		vector<Point> approx;
		approxPolyDP(Mat(contours[ith]), approx, arcLength(Mat(contours[ith]), true)*.02, true);
		switch(approx.size()) {
			case 4:
				drawContours(frame, contours, ith, CV_RGB(55, 0, 200), -1);
				break;
			default:
			//	if (approx.size() > 6) {
			//		drawContours(frame, contours, ith, CV_RGB(103,227,0), -1);
			//	}
				break;
		}
	}
	imshow("image raw", frame);
}

unsigned short distance_to_tools(unsigned short width) {
	return(373.768293*(pow(width,-0.982146)));
}

void find_circle() {
	std::vector<Vec3f> circles;
	HoughCircles(grey, circles, CV_HOUGH_GRADIENT, 2, grey.rows/4, 200, 100);

	// find the smallest circle
	unsigned short min_radius = -1;
	int smallest_circle = -1;
	for (size_t r = 0; r < circles.size(); r++) {
		int radius = cvRound(circles[r][2]);
		if (radius < min_radius) {
			min_radius = radius;
			smallest_circle = r;
		}
	}

	/* this draws every circle 
	for (size_t j = 0; j < circles.size(); j++) {
		int radius = cvRound(circles[j][2]);
		Point center(cvRound(circles[j][0]), cvRound(circles[j][1]));
	//	circle( frame, center, 3, CV_RGB(200, 55, 0), -1, 8, 0);
		std::stringstream ss;
		ss << j;
		string output;
		ss >> output;
		putText(frame, output, center, FONT_HERSHEY_PLAIN, 1.0, CV_RGB(190, 23, 30), 2, CV_AA);
		circle( frame, center, radius, Scalar(55, 200, 0), 3, 8, 0 );
	}	*/

	/* this writes the radius for each circle
	for (size_t i = 0; i < circles.size(); i++) {
		std::stringstream ss;
		ss << "radius" << ' ' << '(' << i << "):" << ' ' << cvRound(circles[i][2]);
		string output(ss.str());
		
		putText(frame, output, Point(10, (470 - (15*i))), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(190, 23, 30), 2, CV_AA);
	}	*/
	// distance finding. taking out - superfluous
/*	if (smallest_circle > 0) {
		Point center(cvRound(circles[smallest_circle][0]), cvRound(circles[smallest_circle][1]));
		circle( frame, center, min_radius, Scalar(55, 200, 0), 3, 8, 0);
		std::stringstream ss;
		ss << "distance:" << ' ' << distance_to_tools(min_radius);
		string output(ss.str());
		putText(frame, output, Point( 10, 470 ), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(23, 190, 30), 2, CV_AA);
	}	*/
	imshow("image raw", frame);
}

int main(int argc, char* argv[]) {
	if (argc != 2) {
		std::cerr << "you must include a shape to look for\n";
		std::cerr << "\t(t)riangle, (c)ircle, or (s)quare\n";
		return(20);
	}


	VideoCapture cap(0);
	if (!cap.isOpened()) {
		std::cerr << "unable to open camera\n";
		return(10);
	}
	lo_thresh = 50;
	hi_thresh = 400;
	namedWindow("image raw", CV_WINDOW_AUTOSIZE);
	namedWindow("image grey", CV_WINDOW_AUTOSIZE);
	namedWindow("image edges", CV_WINDOW_AUTOSIZE);

	void (*call)() = NULL;
	char desired(argv[1][0]);
	switch (desired) {
		case 't':
			call = find_triangle;
			break;
		case 's':
			call = find_square;
			break;
		case 'c':
			call = find_circle;
			break;
		default:
			std::cerr << "cannot determine object to find. try again\n";
			return(30);
	}

	for (;;) {
		cap >> frame;
		imshow("image raw", frame);
	
		cvtColor(frame, grey, CV_RGB2GRAY);
		imshow("image grey", grey);

		Canny(grey, edges, lo_thresh, hi_thresh);
		imshow("image edges", edges);

		call();

		if (waitKey(30) == 27) {
			break;
		}
	}

	return(0);
}
