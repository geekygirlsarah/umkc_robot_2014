#include <highgui.h>
#include <cv.h>
#include <iostream>
#include <string>
#include <vector>

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
	Canny(grey, edges, lo_thresh, hi_thresh);
	imshow("image edges", edges);
	find_shapes();
}	

int main(int argc, char* argv[]) {
	VideoCapture cap(0);
	if (!cap.isOpened()) {
		std::cerr << "unable to open camera\n";
		return(1);
	}
//	string filename(argv[1]);
	lo_thresh = 50;
	hi_thresh = 400;
	namedWindow("image raw", CV_WINDOW_AUTOSIZE);
	namedWindow("image grey", CV_WINDOW_AUTOSIZE);
	namedWindow("image edges", CV_WINDOW_AUTOSIZE);
		string slider_lo = "threshold lo";
		string slider_hi = "threshold hi";
		createTrackbar( slider_lo, "image edges", &lo_thresh, max_thresh, slider_lo_thresh );
		createTrackbar( slider_hi, "image edges", &hi_thresh, max_thresh, slider_hi_thresh );

	for (;;) {
		cap >> frame;
		imshow("image raw", frame);
	
		cvtColor(frame, grey, CV_RGB2GRAY);
		imshow("image grey", grey);

		slider_lo_thresh(lo_thresh, 0);
		slider_hi_thresh(hi_thresh, 0);

	//	while (waitKey(10) != 27);
		if (waitKey(30) == 27) {
			break;
		}
	}

	return(0);
}
