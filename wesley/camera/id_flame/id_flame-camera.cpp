/* ID_FLAME-CAMERA.CPP
 * written by: Eric M Gonzalez, Chase Peterson
 * date 17FEB14
 *
 * PURPOSE: this code will identify the burning rig with less than
 * 			0.0500 confidence (this is better) from a stationary,
 * 			pre-determined viewing position. servos are at:
 *
 *				 10    120     110     15      90      90
 *
 * 			whether this is hard-coded in the arm or requested by a
 * 			call to put(X, Y, Z, P) is up to the arm team.
 *
 * 			at this position, it should be able to see all 3 rigs.
 * 			once the camera has reached this point, it will take a
 * 			snapshot of the field.
 *
 * 			then, once for each rig shape, this code will interest
 * 			itself in only a region of interest for each shape. it
 * 			will then compare a pre-captured image against that 
 * 			region of interest. the pre-capture image contains both
 * 			the flame and the rig shape for increased confidence.
 *
 * 			using matchTemplate(,,,CV_TM_SQDIFF_NORMED), a resultant
 * 			matrix will be obtained with the confidence that the pre-
 * 			captured image is in the section. after testing and some
 * 			tweaking, it was determined that a confidence value of
 * 			less than .1000 is satisfactory for identifying the burning
 * 			rig. it was further observed that a confidence of less than
 * 			.0500 is also a highly confident identification. any value
 * 			above these two values is not confident enough and will
 * 			result in returning 0 (no rig found).
 *
 * RECOMMENDED BEHAVIOR:
 * 			the return value of this program is that of the matched rig.
 * 			these values are:
 *
 * 				0 - NONE
 * 				1 - SQUARE
 * 				2 - TRIANGLE
 * 				3 - CIRCLE
 *
 * 			the caller should look at these values to determine what to
 * 			do next.
 *
 * 			on 0 - the caller should reposition the robot or camera and
 * 			attempt the identification again. this should be repeated
 * 			at least 5 times.
 *
 * 			on 1, 2, or 3 - the caller should pass the identified rig
 * 			to the tool identification program.
 *
 * 			on 10 or greater - these return values indicate failure. the
 * 			caller should identify the failure, display an LED array code,
 * 			and write a descriptive error to a log on disk.
 */

#include <cv.h>			// base opencv library
#include <highgui.h>	// videocapture, imread, imwrite

using namespace cv;

#include <iostream>		// base I/O
#include <sys/stat.h>	// stat() - file presence

int main(int argc, char* argv[]) {
	// for readability of code
	enum rig_shapes { NONE, SQUARE, TRIANGLE, CIRCLE };
	short matched_rig = NONE;

	// create a capture frame of the playfield for working on.
	Mat frame;
	VideoCapture capture(0);
	if (!capture.isOpened()) {
		std::cerr << "unable to open camera device. this is fatal - bailing" << std::endl;
		return(50);
	}
    // this throws a SEGFAULT when uncommented. not sure why, but leaving it
    //    out is harmless - just a waste of a few resources while this runs.
    //    the camera will be released by the destructor when main returns.
//	capture.release();
	// convert captured frame to grayscale and run through theshold to create
	//    a binary image for higher confidence
	cvtColor(frame, frame, CV_RGB2GRAY);
	threshold(frame, frame, 80, 255, CV_THRESH_BINARY);

	// create an array of the pre-captured rig images.
	// verify that all files are extant - fail otherwise
	//
	// these images should be binary images. if not, running them
	//    through threshold is a small matter of adding the code.
	Mat rigs[4];
	struct stat filecheck;
	if ( stat("./rig_square.png", &filecheck) == 0 ) {
		rigs[SQUARE] = imread("./rig_square.png", CV_LOAD_IMAGE_GRAYSCALE);
	} else {
		std::cerr << "cannot find rig_square.png\n";
		return(10);
	}
	if ( stat("./rig_square.png", &filecheck) == 0 ) {
		rigs[TRIANGLE] = imread("./rig_triangle.png", CV_LOAD_IMAGE_GRAYSCALE);
	} else {
		std::cerr << "cannot find rig_triangle.png\n";
		return(20);
	}
	if ( stat("./rig_square.png", &filecheck) == 0 ) {
		rigs[CIRCLE] = imread("./rig_circle.png", CV_LOAD_IMAGE_GRAYSCALE);
	} else {
		std::cerr << "cannot find rig_circle.png\n";
		return(30);
	}
	// an alternative method identifying the burning rig. this is
	//    significantly less confident than the one implemented.
	//    be sure to alter the matchTemplate() call in the for-loop
	//    below.
//	Mat flame = imread("rig_flame.png", CV_LOAD_IMAGE_GRAYSCALE);

	// an array of rectangles to create the regions of interest during
	//    the for-loop. notice that ROI_rigs[NONE] has no shape.
	Rect ROI_rigs[4] = {
		Rect(  0,   0,   0,   0),	// here there be MONSTERS
		Rect( 20, 160, 140, 240),	// SQAURE is in this area
		Rect(300, 160, 140, 240),	// TRIANGLE is here
		Rect(480, 160, 140, 240)	// CIRCLES live here
	};
	
	// for us to even consider a confidence value, it MUST be better
	//    than 1.0 - ignore all higher values.
	double best_confidence = 1.0f;
	// for each shape, trim the frame down to that shapes pre-determined
	//    region of interest and then compare that against the pre-
	//    captured image of a rig.
	for (short shape = SQUARE; shape <= CIRCLE; shape++) {
		// crop the frame
		Mat viewport = frame(ROI_rigs[shape]);
		// resultant matrix for matchTemplate()
		Mat result;
		matchTemplate(viewport, rigs[shape], result, CV_TM_SQDIFF_NORMED);
	//	matchTemplate(viewport, flame, result, CV_TM_SQDIFF_NORMED);

		// in the resultant matrix, darker values are higher confidence.
		//
		// for each shape, find what the highest confidence is (the lowest
		//    value found in the matrix). if lower than either our limit or
		//    a previous best_confidence, store this information.
		double minVal;
		minMaxLoc(result, &minVal);
		if (minVal < best_confidence) {
			best_confidence = minVal;
			matched_rig = shape;
		}

		// for debugging - can safely ignore
	//	std::cout << shape << ": " << minVal << std::endl;
	}

	// for debugging - can safely ignore
//	std::cout << "best confidence: " << best_confidence << std::endl;

	// during testing using the array of pre-captured images, it was
	//    found that a confidence found of less than .0500 was good
	//    enough to determine the burning rig. if you desire a safer
	//    value (due to light constraints), .1000 is also a good value.
	if (best_confidence < 0.050000) {
		// this output block allows the user to see what was found
		std::cout << "WESLEY :: id_flame --> I see a ";
		switch(matched_rig) {
			case SQUARE:
				std::cout << "SQUARE";
				break;
			case TRIANGLE:
				std::cout << "TRIANGLE";
				break;
			case CIRCLE:
				std::cout << "CIRCLE";
				break;
			default:
				std::cout << "MISTAKE -- should not have gotten here.";
				break;
		}
		std::cout << std::endl;
	} else {
		// if the confidence was not within our tolerance, reset
		//    matched_rig to NONE and continue. 
		matched_rig == NONE;
	}
	// return matched_rig back to caller.
	return(matched_rig);
}
