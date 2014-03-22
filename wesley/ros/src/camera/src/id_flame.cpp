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
 * 			on > 10 - these return values indicate failure. the caller
 * 			should identify the failure, display an LED array code, and
 * 			write a descriptive error to a log on disk.
 */

#include <cv.h>			// base opencv library
#include <highgui.h>	// videocapture, imread, imwrite

using namespace cv;

#include <iostream>		// base I/O
#include <sys/stat.h>	// stat() - file presence

int main(int argc, char* argv[]) {
	// in production code, this program will have no arguments.
//	if (argc != 2) {
//		std::cerr << argv[0] << " <frame to test>" << std::endl;
//		return(50);
//	}

	enum rig_shapes { NONE, SQUARE, TRIANGLE, CIRCLE };
	short matched_rig = NONE;

	Mat frame;
//	if (strcmp(argv[1], "--camera") == 0) {
		VideoCapture capture(0);
		if (!capture.isOpened()) {
			std::cerr << "unable to open camera device. this is fatal - bailing" << std::endl;
			return(50);
		}

		capture >> frame;
		cvtColor(frame, frame, CV_RGB2GRAY);
	//	capture.release();
/*	} else {
		std::string filename(argv[1]);
		struct stat verify;
		if (stat(filename.c_str(), &verify) == 0) {
			frame = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);
		} else {
			std::cerr << "unable to load frame image. this is fatal - bailing" << std::endl;
			return(60);
		}
	}
*/	threshold(frame, frame, 80, 255, CV_THRESH_BINARY);

	Mat rigs[4];
	struct stat verify;
	if (stat("./rig_square.png", &verify) == 0) {
		rigs[SQUARE] = imread("./rig_square.png", CV_LOAD_IMAGE_GRAYSCALE);
	} else {
		std::cerr << "could not locate file 'rig_square.png' - fatal; bailing." << std::endl;
		return(10);
	}
	if (stat("./rig_triangle.png", &verify) == 0) {
		rigs[TRIANGLE] = imread("./rig_triangle.png", CV_LOAD_IMAGE_GRAYSCALE);
	} else {
		std::cerr << "could not locate file 'rig_triangle.png' - fatal; bailing." << std::endl;
		return(20);
	}
	if (stat("./rig_circle.png", &verify) == 0) {
		rigs[CIRCLE] = imread("./rig_circle.png", CV_LOAD_IMAGE_GRAYSCALE);
	} else {
		std::cerr << "could not locate file 'rig_circle.png' - fatal; bailing." << std::endl;
		return(30);
	}

//	Mat flame = imread("rig_flame.png", CV_LOAD_IMAGE_GRAYSCALE);
/*	Rect ROI_rigs[4] = {
		Rect(  0,   0,   0,   0),
		Rect( 20, 160, 140, 240),
		Rect(300, 160, 140, 240),
		Rect(480, 160, 140, 240)
	};
*/
	Rect ROI_rigs[4] = {
		Rect(  0 ,   0,  0,   0),
		Rect( 20, 120, 180, 320),
		Rect(300, 120, 180, 320),
		Rect(480, 120, 160, 320)
	};

	double best_confidence = 1.0f;
	for (short shape = SQUARE; shape <= CIRCLE; shape++) {
		Mat viewport = frame(ROI_rigs[shape]);
		Mat result;
		matchTemplate(viewport, rigs[shape], result, CV_TM_SQDIFF_NORMED);
	//	matchTemplate(viewport, flame, result, CV_TM_SQDIFF_NORMED);

		double minVal, maxVal;
		Point minLoc, maxLoc;
		minMaxLoc(result, &minVal);
		if (minVal < best_confidence) {
			best_confidence = minVal;
			matched_rig = shape;
		}

		std::cout << shape << ": " << minVal << std::endl;

		// display both viewport and result to user, wait to continue
	/*	namedWindow("viewport", CV_WINDOW_AUTOSIZE);
		imshow("viewport", viewport);
		namedWindow("result", CV_WINDOW_AUTOSIZE);
		imshow("result", result);
		while(waitKey() != 27);
		destroyAllWindows();	*/
	//*/
	}

//	std::cout << "best confidence: " << best_confidence << std::endl;
	// .025 is STRONGLY confident
	// .050 is moderately confidenct.
	// .075 is adequate.
	// or .1000 if you want to be safe
	if (best_confidence < 0.075000) {
		std::cout << "WESLEY :: id_flame --> I see a ";
		switch(matched_rig) {
			case SQUARE:
				std::cout << "SQUARE";
			//	matched_rig = SQUARE;
				break;
			case TRIANGLE:
				std::cout << "TRIANGLE";
			//	matched_rig = TRIANGLE;
				break;
			case CIRCLE:
				std::cout << "CIRCLE";
			//	matched_rig = CIRCLE;
				break;
			default:
				std::cout << "MISTAKE -- should not have gotten here.";
				break;
		}
		std::cout << std::endl;
	} else {
		std::cout << "no confidence." << std::endl;
		matched_rig = NONE;
	}
	return(matched_rig);
}
