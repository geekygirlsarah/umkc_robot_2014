/** THRESHOLD_SLIDER.CPP
 *  by: Eric M Gonzalez
 * 
 *  PURPOSE: This code provides a side-by-side view of what the camera sees and
 *           the same frame after a run through OpenCV's threshold(). A slider
 *           is available at the top of the window to adjust the hi_ and lo_
 *           threshold values given to threshold.
 * 
 *           This allows the user to modify the threshold() values used through-
 *           out the cv code to help fine-tune what is seen and used by the camera.
 *           this can help to clean up noise in the camera's images.'
 * 
 */

#include <highgui.h>
#include <cv.h>
#include <iostream>
#include <string>
#include <vector>

using namespace std;
using namespace cv;

Mat frame;
Mat thresh;
Mat grey;

int lo_thresh;
int hi_thresh;
int max_thresh = 255;
int min_thresh = 0;

void slider_lo_thresh( int, void* ) {
	threshold(grey, thresh, lo_thresh, hi_thresh, CV_THRESH_BINARY);
//	imshow("image thresh", thresh);
}

void slider_hi_thresh( int, void* ) {
	threshold(grey, thresh, lo_thresh, hi_thresh, CV_THRESH_BINARY);
//	imshow("image thresh", thresh);
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
	namedWindow("image thresh", CV_WINDOW_AUTOSIZE);
		string slider_lo = "threshold lo";
		string slider_hi = "threshold hi";
		createTrackbar( slider_lo, "image thresh", &lo_thresh, max_thresh, slider_lo_thresh );
		createTrackbar( slider_hi, "image thresh", &hi_thresh, max_thresh, slider_hi_thresh );

	cap >> frame;
	// make a second mat twice as wide as the camera frame
	Mat window(frame.rows, frame.cols*2, CV_8UC3);
	for (;;) {
		cap >> frame;
	//	imshow("image raw", frame);
	
		cvtColor(frame, grey, CV_RGB2GRAY);
	//	imshow("image grey", grey);

		slider_lo_thresh(lo_thresh, 0);
		slider_hi_thresh(hi_thresh, 0);

		cvtColor(thresh, thresh, CV_GRAY2RGB);
		// horizontally concatenate the two images into the window Mat.
		hconcat(frame, thresh, window);
		imshow("image thresh", window);
	//	while (waitKey(10) != 27);
		if (waitKey(30) == 27) {
			break;
		}
	}

	return(0);
}
