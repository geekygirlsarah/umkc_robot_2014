/* id_RIGS.cpp
 * written: by Eric M Gonzalez, Chase Peterson
 * date: 15FEB2014
 *
 * PURPOSE: this code examines an image taken at the
 *          following servo positions:
 *          	base:			10
 *          	shoulder:		120
 *          	elbow:			110
 *          	wrist_roll:		10
 *          	wrist_pitch:	95
 *
 *          there will be three pre-captured images that
 *          contain both the flame and the tool hole on
 *          the rig; one for each rig aflame.
 *          
 *          this code will then compare using matchTemplate() the
 *          'square' image with the frame, then the 'triangle, and
 *          then the 'circle'. the first confident match found using
 *          matchTemplate(,,,CV_TM_SQDIFF) will identify the burning
 *          rig. this information will then be passed on to the main
 *          system for further operation.
 *
 */

#include <cv.h>			// opencv main libraries
#include <highgui.h>	// imread and VideoCapture
#include <iostream>		// simple I/O
using namespace cv;
#include <sys/stat.h>	// stat()

int main() {
	enum matches { NONE, SQUARE, TRIANGLE, CIRCLE };
	short matched_rig = NONE;

	// create our templates for comparison
	Mat rigs[4];

	// determine that all the files are in the right spots.
	// quit on failure.
	//
	// desired: have LEDArray indicate error and write error to disk.
	struct stat filecheck;
	if ( stat("./rig_square.png", &filecheck) == 0 ) {
		rigs[SQUARE] = imread("./rig_square.png");
	} else {
		std::cerr << "cannot find rig_square.png\n";
		return(10);
	}
	if ( stat("./rig_square.png", &filecheck) == 0 ) {
		rigs[TRIANGLE] = imread("./rig_triangle.png");
	} else {
		std::cerr << "cannot find rig_triangle.png\n";
		return(20);
	}
	if ( stat("./rig_square.png", &filecheck) == 0 ) {
		rigs[CIRCLE] = imread("./rig_circle.png");
	} else {
		std::cerr << "cannot find rig_circle.png\n";
		return(30);
	}

	// position the arm in the needed spot.

	// capture frame from camera - exit on failure.
	//
	// desired: have LEDArray indicate error and write to disk.
	VideoCapture capture(0);
	if (!capture.isOpened()) {
		std::cerr << "unable to open camera device(0)\n";
		return(40);
	}

	Mat frame;
	capture >> frame;
	// during resting, this line was throwing SEGFAULT.
	// it's not necessary to release the camera, but would be nice.
//	capture.release();

	// convert frame to grayscale and run through a
	//    threshold to convert to binary
	cvtColor(frame, frame, CV_RGB2GRAY);
	threshold(frame, frame, 80, 255, CV_THRESH_BINARY);
	bitwise_xor(frame, Scalar(255, 0, 0), frame);

	for (short shape = SQUARE; shape <= CIRCLE; shape += 1) {
		cvtColor(rigs[shape], rigs[shape], CV_RGB2GRAY);
		threshold(rigs[shape], rigs[shape], 80, 255, CV_THRESH_BINARY);
		bitwise_xor(rigs[shape], Scalar(255, 0, 0), rigs[shape]);
	}

	// for each of rig_[square|circle|triangle] compare against the frame
	// and then evaluate the resultant Mat for any confident matches.
	// when a confident match is found, mark it and then return.

	// owing to simple test data, the check for a confident result will
	//    be any value returned by minMaxLoc() that is less than 1000.
	double minimal_match_value = 9999.0f;
	for (short shape = SQUARE; shape <= CIRCLE; shape += 1) {
		// don't want to have a left over result matrix conflicting
		//    with each search, so this is recreated every loop.
		Mat result;

		// CV_TM_SQDIFF returns solid 'yes'/'no' confidence in the image.
		//    this is good IF you have _exactly_ what you are looking for.
		// CV_TM_SQDIFF_NORMED returns a more 'fuzzy' confidence value,
		//    and so is better if you have what you want, but not in the
		//    exact configuration.
		//
		// the result matrix is scaled by a length/width factor equivalent
		//    to the size of the matching template. it is recommended to
		//    have each match template the same size so that translation
		//    equations are the same for each shape.
		matchTemplate(frame, rigs[shape], result, CV_TM_SQDIFF_NORMED);


		// do magic on result to find confidence
		// for CV_TM_SQDIFF_NORMED, running minMaxLoc() on the resultant
		//    matrix and running a min_value for loop will return the object
		//    found with the best confidence and a Point() containing
		//    the co-ordinates of that confidence WITH RESPECT TO the
		//    restulant matrix.
		// as with the fresh result matrix, recreate these on each loop to
		//    avoid data corruption
		double minVal, maxVal;
		Point match_min, match_max;
		minMaxLoc(result, &minVal, &maxVal, &match_min, &match_max);

		// after testing with match_flame.cpp, a value less than 1000 (or
		//    even 1000000) is a confident result. don't need location,
		//    but will write it to a file anyway. it will need to have a
		//    translation based on a scale proportional to the rig_<shape>
		//    image to identify location inside frame.
		if (minVal < 1000) {
			minimal_match_value = minVal;
			matched_rig = shape;
			// found a confident result. store it and break back to main
			break;
		}
	}
	
	// all work finished. return the found shape to the caller.
	//
	// return value: 0 - no object found
	//               1 - SQUARE found
	//               2 - TRIANGLE found
	//               3 - CIRCLE found
	// desired: indicaet status on LEDArray and write to disk.
	return(matched_rig);
}
