/* ellipse.cpp
 *
 * this file is a port of code found at:
 * http://stackoverflow.com/questions/4785419/detection-of-coins-and-fit-ellipses-on-an-image
 *
 * Nothing wrong with that code. It works. It can be found in the file:
 *     ellipse_orig.cpp
 *
 * The code there has been modified to accept a stream from a VideoCapture
 *     object instead of a single binary image (as provided in the question
 *     at the above address.
 *
 * This code will attempt to modify to code away from using IplImage arrays
 *    and use the Mat object. This code will also modify the values to the
 *    various functions used in an attempt to reduce the false-positives that
 *    have been detected using the original code.
 *
 * ported by: Eric M Gonzalez.
 * date: 06 FEB 2014
 *
 */

#include <cv.h>			// opencv core
#include <highgui.h>	// opencv gui structure

#include <iostream>		// basic (in/out)put functionality

using namespace cv;		// the majority of code will be
						//     drawn from this namespace

// This needs to be high to avoid finding false-positives in small detected
//    objects. This could be a good place to start for honing in on correct
//    identification.
// Need to identify how far the capture will be from object and what the
//    center-hole's apparent size will be. That info will help alot.
#define MIN_AREA 100.00

// emg -- from original code:
//
// One way to tell if an object is an ellipse is to look at the relationship
// of its area to its dimensions.  If its actual occupied area can be estimated
// using the well-known area formula Area = PI*A*B, then it has a good chance of
// being an ellipse.
//
// This value is the maximum permissible error between actual and estimated area.
//
#define MAX_TOL 100.00

int main(int argc, char* argv[]) {
	// calling the constructor with a "string filename" failes at runtime.
	//    stick to calling object(0), where 0 is first capture device found
	VideoCapture capture(0);		// open specific device in system
											// using UDEV rules we can specify a name
											//    as in the case of /dev/camera
	if (!capture.isOpened()) {
		std::cerr << "unable to open capture at /dev/capture.\n";
		std::cerr << "verify UDEV rules, or modify code to get rid of hard-coded value\n\n";
		return(1);		// errorcode 1: invalid capture interface
	}
	
//	window.row(1);
	Mat frame;
	capture >> frame;
	Mat camera;
	Mat thresh(frame.size(), 1);				// copy of camera frame at 1 channel
	Mat edges(frame.size(), 1);				// copy of camera frame at 1 channel
	Mat dilat(frame.size(), 1);				// copy of camera frame at 1 channel
	Mat windowH( frame.rows, frame.cols*2, CV_8UC3);		// a frame, 2x capture picture wide
	Mat windowL( frame.rows, frame.cols*2, CV_8UC3);		// a frame, 2x capture picture wide
	Mat window( frame.rows*2, frame.cols*2, CV_8UC3);		// a frame, 2x capture picture wide
	// for testing, it's a good idea to display a window for each segment chunk
	//    of overall algorithm being worked on.
	// create a window to display raw capture frame;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	namedWindow("window", CV_WINDOW_AUTOSIZE);

	// create a series of trackbars to adjust function values
	string slider_thresh_hi = "threshold hi";
	string slider_thresh_lo = "threshold lo";
	unsigned int keypress = 0;				// store keypresses inside the GUI window
	#define EVER ;;
	for (EVER) {
		capture >> camera;
	//	camera.copyTo(thresh);
		// convert the camera frame into grayscale for any use
		cvtColor(camera, thresh, CV_RGB2GRAY);
	
		// magic happens here
		// want to add two sliders for thresh hi and thresh lo
		threshold(thresh, thresh, 80, 255, CV_THRESH_BINARY);
	//	dilate(thresh, dilat, 0, Point(-1, -1), 2);
		bitwise_xor(thresh, Scalar(255, 0, 0), dilat);
		Canny(thresh, edges, 50, 400);
		findContours(thresh, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
		
		// contours are found. let's narrow down what we want.
		Mat mask = Mat::zeros(frame.size(), CV_8UC1);
		drawContours(mask, contours, -1, CV_RGB(0xFF, 0x66, 0x99), CV_FILLED);

		// draw largest connected contour - is this what I want? probably.
	/*	vector<double> areas(contours.size());
		for(int i = 0; i < contours.size(); i++) {
			areas[i] = contourArea(Mat(contours[i]));
		}
		double max;
		Point maxPosition;
		minMaxLoc(Mat(areas),0,&max,0,&maxPosition);
		drawContours(mask, contours, maxPosition.y, Scalar(1), CV_FILLED);
	*/

	
		// before cat'ing the image into the larger window, convert it back to a matching RGB colorspace
		cvtColor(thresh, thresh, CV_GRAY2RGB);
		cvtColor(edges, edges, CV_GRAY2RGB);
		cvtColor(mask, mask, CV_GRAY2RGB);
	//	camera.copyTo(window(Rect(0, 0, camera.cols, camera.rows)));
		hconcat(camera, edges, windowH);
		hconcat(thresh, mask, windowL);
		vconcat(windowH, windowL, window);
		imshow("window", window);
		keypress = waitKey(30);
		if (keypress == 27) {				// [ESC]
			break;							// 'quit'
		}
	}

	destroyAllWindows();
	return(0);
}
