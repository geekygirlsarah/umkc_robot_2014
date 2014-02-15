#include <highgui.h>
#include <cv.h>
#include <iostream>
#include <string>
#include <vector>

using namespace std;
using namespace cv;

Mat frame;
Mat thresh;

int main(int argc, char* argv[]) {
	VideoCapture cap(0);
	if (!cap.isOpened()) {
		std::cerr << "unable to open camera\n";
		return(1);
	}
//	string filename(argv[1]);
	namedWindow("image raw", CV_WINDOW_AUTOSIZE);
	namedWindow("image thresh", CV_WINDOW_AUTOSIZE);

	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	cap >> frame;
	Mat window(frame.rows, frame.cols*2, CV_8UC3);
	Mat viewport;
	for (;;) {
		cap >> frame;
	//	imshow("image raw", frame);
	
		cvtColor(frame, thresh, CV_RGB2GRAY);
	//	imshow("image grey", grey);
		viewport = frame(Rect(0, 150, frame.cols, 200));
		
		threshold(thresh, thresh, 60, 255, CV_THRESH_BINARY);
		bitwise_xor(thresh, Scalar(255, 0, 0), thresh);
		Canny(thresh, thresh, 50, 400, 5);
	//	findContours(thresh, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
		findContours(thresh, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

	/*	vector<double> areas(contours.size());
		for(int i = 0; i < contours.size(); i++) {
			areas[i] = contourArea(Mat(contours[i]));
		}
		double max;
		Point maxPosition;
		minMaxLoc(Mat(areas),0,&max,0,&maxPosition);
		drawContours(frame, contours, maxPosition.y, CV_RGB(0xFF, 0x66, 0x99), CV_FILLED);
	*/
		drawContours(frame, contours, -1, CV_RGB(0xFF, 0x66, 0x99), CV_FILLED, CV_AA);
		cvtColor(thresh, thresh, CV_GRAY2RGB);
		hconcat(frame, thresh, window);

		imshow("image raw", viewport);
		imshow("image thresh", window);
	//	while (waitKey(10) != 27);
		if (waitKey(30) == 27) {
			break;
		}
	}

	return(0);
}
