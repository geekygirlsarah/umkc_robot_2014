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
#include <fstream>		// basic file I/O
#include <vector>		// magical arrays
#include <sys/stat.h>	// stat() - file presence

#include <ros/ros.h>
#include <wesley/arm_point.h>

bool waiting = true;
void arm_block_wait(const wesley::arm_point& msg) {
	ROS_INFO("ID_FLAME :: arm_block_wait --> got response from arm.");
	// do some more appropriate response checking
	waiting = false;
	// the message arrives before the arm is physically done moving.
	//    wait one second to allow arm to settle.
	sleep(1);
}

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "id_flame");
	ROS_INFO("ID_FLAME --> initializing.");

	// in production code, this program will have no arguments.
//	if (argc != 2) {
//		std::cerr << argv[0] << " <frame to test>" << std::endl;
//		return(50);
//	}

	enum rig_shapes { NONE, SQUARE, TRIANGLE, CIRCLE };
	short matched_rig = NONE;

	Mat frame;
	VideoCapture capture(0);
	if (!capture.isOpened()) {
		ROS_ERROR("unable to open camera device. this is fatal - bailing");
		return(50);
	}
	ROS_INFO("ID_FLAME --> camera open.");

	Mat rigs[4];
	
	string path = "/home/umkc/wesley/umkc_robot_2014_arduino/wesley/config/";
	string image_file[4] = {
		"",
		"rig_square.png",
		"rig_triangle.png",
		"rig_circle.png"
	};
	string filename = "";

	// these should eventually be configurable via a launch file
	struct stat verify;
	filename = path + image_file[SQUARE];
	if (stat(filename.c_str(), &verify) == 0) {
		rigs[SQUARE] = imread(filename.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
	} else {
		ROS_ERROR("could not locate file '%s' - fatal; bailing.", image_file[SQUARE].c_str());
		return(10);
	}
	ROS_INFO("ID_FLAME --> '%s' found.", image_file[SQUARE].c_str());
	filename = path + image_file[TRIANGLE];
	if (stat(filename.c_str(), &verify) == 0) {
		rigs[TRIANGLE] = imread(filename.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
	} else {
		ROS_ERROR("could not locate file '%s' - fatal; bailing.", image_file[TRIANGLE].c_str());
		return(10);
	}
	ROS_INFO("ID_FLAME --> '%s' found.", image_file[TRIANGLE].c_str());
	filename = path + image_file[CIRCLE];
	if (stat(filename.c_str(), &verify) == 0) {
		rigs[CIRCLE] = imread(filename.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
	} else {
		ROS_ERROR("could not locate file '%s' - fatal; bailing.", image_file[CIRCLE].c_str());
		return(10);
	}
	ROS_INFO("ID_FLAME --> '%s' found.", image_file[CIRCLE].c_str());

	std::ifstream fin;
	filename = path + "position_rigs.lst";
	if (stat(filename.c_str(), &verify) == 0) {
		fin.open(filename.c_str(), std::ifstream::in);
	} else {
		ROS_ERROR("could not locate file 'position_rigs.lst' - fatal; bailing.");
		return(40);
	}
	ROS_INFO("ID_FLAME --> 'position_rigs.lst' found.");

	vector<wesley::arm_point> hopper;
	int x, y, z, p, r;
	while (fin >> x >> y >> z >> p >> r) {
		wesley::arm_point point;
		point.direct_mode = true;
		point.x = x;
		point.y = y;
		point.z = z;
		point.p = p;
		point.r = r;
		point.cmd = "put_point";

		hopper.push_back(point);
	}
	fin.close();
	ROS_INFO("ID_FLAME --> hopper filled with %d positions.", hopper.size());

	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<wesley::arm_point>("/arm/put/point", 1000);
	ros::Subscriber sub = nh.subscribe("/arm/response", 1000, &arm_block_wait);
	ROS_INFO("ID_FLAME --> PUB and SUB created.");

	while (pub.getNumSubscribers() <= 0) {
		//  wait until something is attached to the publisher.
	}
	ROS_INFO("ID_FLAME --> number of subscribers: (%d)", pub.getNumSubscribers());

	wesley::arm_point carry;
	carry.direct_mode = false;
	carry.cmd = "carry";

	short position = 0;
//	pub.publish(carry);
//	do {
//		ros::spinOnce();
//	} while (waiting);

	ROS_INFO("ID_FLAME --> begin loop");
	do {
		// reset waiting variable
		waiting = true;
		// move arm to a position in the list
		pub.publish(hopper[position]);
		ROS_INFO("ID_FLAME :: do --> position published");
		do {
			// activates callbacks and mesage delivery queue.
			// publishes, and waits for /arm/response to receive
			//    a message.
			ros::spinOnce();
		} while(waiting);

		capture >> frame;
		cvtColor(frame, frame, CV_RGB2GRAY);
		threshold(frame, frame, 80, 255, CV_THRESH_BINARY);

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
		// .050 is moderately confident.
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

		position++;
		if (position >= hopper.size()) {
			ROS_INFO("ID_FLAME --> ran out of positions in the hopper");
			return(60);
		}
	} while(matched_rig == NONE);
	{
		// whether we found a rig or not, return to the carry position.
		pub.publish(carry);
		ros::spinOnce();
	}
	return(matched_rig);
}
