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
	namedWindow("match_frame", CV_WINDOW_AUTOSIZE);
	namedWindow("image thresh", CV_WINDOW_AUTOSIZE);

	Mat match_frame;
	vector<vector<Point> > contours_match;
	if(argc == 2) {
		string filename = argv[1];
		match_frame = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);
	//	bitwise_xor(match_frame, Scalar(255, 0, 0), match_frame);
		findContours(match_frame, contours_match, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
		cvtColor(match_frame, match_frame, CV_GRAY2RGB);
		drawContours(match_frame, contours_match, -1, CV_RGB(0x77, 0x46, 0xD7), CV_FILLED);
		std::stringstream ss;
		ss << contours_match.size();

		putText(match_frame, ss.str(), Point(10, 176), FONT_HERSHEY_PLAIN, 1.5, CV_RGB(0x77, 0x46, 0xD7), 2, CV_AA);
	}

	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	cap >> frame;
	Mat window(frame.rows, frame.cols*2, CV_8UC3);
	Mat viewport;
	for (;;) {
		cap >> frame;
	//	imshow("image raw", frame);
	
	//	cvtColor(frame, thresh, CV_RGB2GRAY);
	//	imshow("image grey", grey);
		viewport = frame(Rect(0, 150, frame.cols, 250));
		cvtColor(viewport, thresh, CV_RGB2GRAY);
		
		threshold(thresh, thresh, 60, 255, CV_THRESH_BINARY);
		bitwise_xor(thresh, Scalar(255, 0, 0), thresh);
		Canny(thresh, thresh, 50, 400, 5);
		imshow("image thresh", thresh);
		findContours(thresh, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
	//	findContours(thresh, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

	//	vector<double> areas(contours.size());
	//	for(int i = 0; i < contours.size(); i++) {
	//		areas[i] = contourArea(Mat(contours[i]));
	//	}
	//	double max;
	//	Point maxPosition;
	//	minMaxLoc(Mat(areas),0,&max,0,&maxPosition);
	//	drawContours(frame, contours, maxPosition.y, CV_RGB(0xFF, 0x66, 0x99), CV_FILLED);
	//	drawContours(frame, contours, maxPosition.y, CV_RGB(0xFF, 0x66, 0x99), CV_FILLED, CV_AA, hierarchy, INT_MAX, Point(0, 150));
	

		size_t closest_match = 0;
		double match_value = 0.0f;
		double match_value_tmp = 0.0f;
		for (size_t ith = 0; ith < contours.size(); ith++) {
			match_value_tmp = matchShapes(contours_match[0], contours[ith], CV_CONTOURS_MATCH_I1, 0.0f);
			if (match_value_tmp > match_value) {
				match_value = match_value_tmp;
				closest_match = ith;
			}
		}
		if (match_value < 1) {
			std::cout << closest_match << ": :: " << match_value << std::endl;
			drawContours(frame, contours, closest_match, CV_RGB(0xFF, 0x66, 0x99), CV_FILLED, CV_AA, hierarchy, INT_MAX, Point(0, 150));
		}
		cvtColor(thresh, thresh, CV_GRAY2RGB);
	//	hconcat(frame, thresh, window);

		imshow("image raw", viewport);
		if (argc == 2) {
			imshow("match_frame", match_frame);
		}
	//	imshow("image thresh", frame);
	//	while (waitKey(10) != 27);
		if (waitKey(30) == 27) {
			break;
		}
	}

	return(0);
}
