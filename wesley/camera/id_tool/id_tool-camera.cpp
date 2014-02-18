#include <cv.h>
#include <highgui.h>

using namespace cv;

#include <iostream>

int main(int argc, char* argv[]) {
	char which_tool = 'a';
	if (argc == 2) {
		which_tool = argv[1][0];
	}

	VideoCapture capture(0);
	if (!capture.isOpened()) {
		std::cerr << "unable to open camera device -- fatal; bailing." << std::endl;
		return(40);
	}

	Mat frame;
	namedWindow("viewport", CV_WINDOW_AUTOSIZE);
	Mat thresh;
	
	Rect ROI_tool = Rect(160, 80, 300, 360);
	// capture frame, convert to grayscale, threshold, and flip
	capture >> frame;

	Mat viewport;
	viewport = frame(ROI_tool);
	viewport.copyTo(thresh);

	cvtColor(thresh, thresh, CV_RGB2GRAY);
	threshold(thresh, thresh, 100, 255, CV_THRESH_BINARY);
	bitwise_xor(thresh, Scalar(255, 0, 0), thresh);

	// setup to run findContours(), then display results
	Mat contour;
	thresh.copyTo(contour);
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(contour, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

	// find contour with largest area - this doesn't mean we've found the tool,
	//    just narrowing it down avoiding shade and shadow
	double max_area = 0.0f;
	int biggest_contour = -1;
	for (size_t ith = 0; ith < contours.size(); ith++) {
		double tmp_area = contourArea(contours[ith]);
		if (tmp_area > max_area) {
			max_area = tmp_area;
			biggest_contour = ith;
		}
	}
//	std::cout << "biggest area: " << max_area << std::endl;

	// approximate a polygon from the found contour
	vector<Point> approx;
	bool tool_found = false;
	if (biggest_contour != -1) {
//	if (biggest_contour != -1 && max_area > 20000) {
		approxPolyDP(contours[biggest_contour], approx, arcLength(contours[biggest_contour], true)*.02, true);
		switch(which_tool) {
			case 't': {
				switch(approx.size()) {
					case 3: case 4: case 5:
						drawContours(viewport, contours, biggest_contour, CV_RGB(0xFF, 0x9A, 0x40), -1);
						putText(frame, "TRIANGLE", Point(20, 460), FONT_HERSHEY_PLAIN, 1.5, CV_RGB(0xFF, 0x9A, 0x40), 2);
						tool_found = true;
						break;
					default:
						break;
				break;
				}
			}
			case 's': {
				switch(approx.size()) {
					case 6: case 7:
						drawContours(viewport, contours, biggest_contour, CV_RGB(0x4B, 0x5C, 0xD7), -1);
						putText(frame, "SQUARE", Point(20, 460), FONT_HERSHEY_PLAIN, 1.5, CV_RGB(0x4B, 0x5C, 0xD7), 2);
						tool_found = true;
						break;
					default:
						break;
				break;
				}
			}
			case 'c': {
				switch(approx.size()) {
					case 8: case 9: case 10:
						drawContours(viewport, contours, biggest_contour, CV_RGB(0xDD, 0x37, 0xB4), -1);
						putText(frame, "CIRCLE", Point(20, 460), FONT_HERSHEY_PLAIN, 1.5, CV_RGB(0xDD, 0x37, 0xB4), 2);
						tool_found = true;
						break;
					default:
						break;
				}
				break;
			}
			default:
				break;
		}
	}
	imshow("frame", frame);
	while (waitKey() != 27);

	return(tool_found);
}
