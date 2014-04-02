/* ID_TOOL-CENTER.CPP
 * written by: Eric M Gonzalez
 * date: 25FEB14
 *
 * PURPOSE: This code follows this algorithm:
 *
 * algorithm:
 *
 *  need to keep track of what the last tool and position we looked at is
 *  	so that if we fail at find_top, we can start back where we left off
 *  	and avoid finding the false-positive again.
 * 	co-ordinate position_tool[5][3]; [ 5 positions ] per [ tool ]
 * 	co-ordinate position_top[3][5]; [ 3 tools ] and [ 5 positions for each ]
 * 	switch(job_state) {
 * 		case (find_tool) {
 * 			for(pos = 1 - 5) {
 * 				for(tool = 1 - 3) {
 * 					position arm at position_tool[pos][tool]
 * 					take a picture
 * 					job_state photo
 * 					if requested tool is found
 * 						set job_state to find_top
 * 						break
 * 					else
 * 						continue
 * 				}
 * 			}
 * 		}
 * 		case (find_top) {
 * 			for (pos = 1 - 5) {
 * 				move arm to position[tool][pos]
 * 				take a picture
 * 				job_state photo
 * 				if top found
 * 					set job_state to id_center
 * 				else
 * 					continue
 * 			}
 * 			if this point is reached, then we did not find top
 * 			set job_state to find_tool
 * 			continue
 * 		}
 * 		case (id_center) {
 * 			use same frame as captureed in find_top
 * 			rejob_state to find the center
 * 			find the distance to the center
 * 			translate between co-ordinate frames
 * 			set job_state to grab_tool
 * 			continue
 * 		}
 * 		case (grab tool) {
 * 			move arm to identified position and pick up tool.
 * 			position arm to CARRY()
 * 			exit , indicating success.
 * 		}
 * 	}
 */

#include <cv.h>			// base OpenCV
#include <highgui.h>	// VideoCapture

// most of this code is done in the OpenCV namespace
using namespace cv;

#include <ros/ros.h>	// base ROS
#include <wesley/arm_point.h>	// custom arm point message

#include <iostream>		// basic I/O
#include <fstream>		// file input
#include <sstream>		// stringstreams
#include <cmath>		// mathematical functions (sqrt)
#include <sys/stat.h>	// file presence verification
#include <unistd.h>		// sleep()

// personal debug defines. set to (0) to turn off.
#define EMGDBG (0)
#define DBGOUT if EMGDBG std::cout
#define DBGCV if EMGDBG

#ifndef M_PI
	#define M_PI 3.141592654
#endif

#ifndef M_PI_2
	#define M_PI_2 1.570796327
#endif

#define wait_on_arm() {				\
			waiting = true;			\
			do {					\
				ros::spinOnce();	\
			} while(waiting);		\
		}
// how to keep track of the tool we're looking for
enum shapes { NONE, SQUARE, TRIANGLE, CIRCLE };
unsigned short tool = NONE;
// how to keep track of what section of the job_state we're in
enum states { FIND_TOOL, FIND_TOP, FIND_DISTANCE, TOOL_GRASP };
unsigned short job_state = FIND_TOOL;
// when we're looking for the tool, this lets us keep track
//    of which tool position we're looking at
enum tool_area { LEFT, CENTER, RIGHT };

// coordinate structure - this will be filled from a file.
// for the oil rigs - use 'coordinates_flame.lst'
// for the tools    - use 'coordinates_tools.lst'
//
// format of the file is:
//    one coordinate per line separated by a space:
//    	 x y z w
//    file does support comments: begin each line of
//       a comment with a #
//struct coordinate {
//	int x;
//	int y;
//	int z;
//	coordinate(int x=0, int y=0, int z=0) : 
//		x(x), y(y), z(z) { };
//};

// quick help in case the program is called incorrectly.
void program_help(const char* arg_zero) {
	std::cerr << "please call " << arg_zero << " with one of:" << std::endl;
	std::cerr << "\t(s or 1), (t or 2), or (c or 3)" << std::endl;
	std::cerr << "\tand the full path location of 'position_tool.lst'" << std::endl;
	std::cerr << "ex: id_tool s /home/umkc/wesley/umkc_robot_2014_arduino/wesley/config/position_tool.lst" << std::endl;
}

bool waiting = true;
void block_arm_wait(const wesley::arm_point& msg) {
	ROS_INFO("ID_TOOL --> got response from arm. unblocking");
	waiting = false;
	// sleep for a few seconds to let the arm settle.
	// have been getting some nasty blur.
	sleep(1);
}

double distance_to_tool(double apparent_area_px, const unsigned short tool=0) {
	double initial_area_px[4] = {
		0.0f,		// no tool
		42452.6,	// SQUARE
		23404.5,	// TRIANGLE
		38908.9,		// CIRCLE
	};

	double initial_distance = 152.4;	// mm, ~6 inches;

	/* Many thanks to Darren for pulling this star out of nowhere.
	 * the z_distance is proportional to a square fraction of the area.
	 *
	 * 		_____
	 * 		| ai
	 * 	    |---  *  dist_inital
	 *	   \| a
	 *
	 */
	return(sqrt(initial_area_px[tool] / apparent_area_px) * initial_distance);
}

// returns index of the biggest contour found within the frame;
// returns -1 if no contour was found.
int process_frame(Mat &frame, Mat &thresh, vector<vector<Point> > &contours) {
	frame.copyTo(thresh);

	cvtColor(thresh, thresh, CV_RGB2GRAY);
	// thresold will reduce the grayscale image to a binary black-and-white
	//    image based on the low_thresh value. values on one side are black,
	//    those on the other are white.
	// the low_thresh value of 80 here was obtained through experimentation.
	//    please use 'threshold_slider' to find other appropriate values.
	threshold(thresh, thresh, 80, 255, CV_THRESH_BINARY);
	// flip the image - black becomes white and vice-versa
	bitwise_xor(thresh, Scalar(255), thresh);

	// pass resultant matrix thresh through findContours()
	findContours(thresh,
				 contours,
	//			 hierarchy,
				 CV_RETR_EXTERNAL,
				 CV_CHAIN_APPROX_NONE);

	// find the largest contour in the frame
	double max_area = 0.0f;
	int biggest_contour = -1;
	for (int ith = 0; ith < contours.size() ; ith++) {
		double tmp_area = contourArea(contours[ith]);
		if (tmp_area > max_area) {
			max_area = tmp_area;
			biggest_contour = ith;
		}
	}

	// returns the index of the biggest contour found, or
	//    -1 if no contours were found.
	return (biggest_contour);
}

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "id_tool");
	
	// this program must be called with one argument (see switch below)
	if (argc != 3) {
		program_help(argv[0]);
		return(10);
	}

	// how were we called?
	switch(argv[1][0]) {
		case 's': case '1':
			tool = SQUARE;
			break;
		case 't': case '2':
			tool = TRIANGLE;
			break;
		case 'c': case '3':
			tool = CIRCLE;
			break;
		default:
			ROS_ERROR("unable to understand called method. -- fatal, bailing.");
			ROS_ERROR("called with: < %d >", argv[1][0]);
			program_help(argv[0]);
			return(20);
			break;
	}

	// position[job_state][area][pos]
	//
	// be sure to fill this with appropriate locations.
	// preferably from a file. see above the coordinate
	// structure for file definition.
	struct stat verify;
	std::ifstream fin;
	string file_position_tool = argv[2];
	if (stat(file_position_tool.c_str(), &verify) == 0) {
		fin.open(file_position_tool.c_str(), std::ifstream::in);
	} else {
		ROS_ERROR("ID_TOOL --> unable to locate %s - fatal; bailing.", file_position_tool.c_str());
		return(60);
	}

	const short MAX_ID_TOOL = 12;

//	wesley::arm_point position[2][3][MAX_ID_TOOL];
//	wesley::arm_point position[area][pos][state]
	wesley::arm_point position[3][MAX_ID_TOOL][2];
	std::string buffer;
	for (int area = LEFT; area <= RIGHT; area++) {
		for (int pos = 0; pos < MAX_ID_TOOL; pos++) {
			for (int state = FIND_TOOL; state <= FIND_TOP;) {
				getline(fin, buffer);
				if (buffer[0] == '#' || buffer.size() == 0) {
					continue;
				}
				std::stringstream ss;
				ss << buffer;
				float x, y, z, p, r;
				ss >> x >> y >> z >> p >> r;
				position[area][pos][state].direct_mode = true;
				position[area][pos][state].x = x;
				position[area][pos][state].y = y;
				position[area][pos][state].z = z;
				position[area][pos][state].p = p;
				position[area][pos][state].r = r;
				switch(state) {
					case FIND_TOOL:
						position[area][pos][state].cmd = "id_tool: FIND_TOOL";
						break;
					case FIND_TOP:
						position[area][pos][state].cmd = "id_tool: FIND_TOP";
						break;
				}
				state++;
			}
		}
	}	//*/
	if (EMGDBG) {
		for (int area = LEFT; area <= RIGHT; area++) {
			for (int pos = 0; pos < MAX_ID_TOOL; pos++) {
				for (int state = FIND_TOOL; state <= FIND_TOP;) {
					ROS_INFO("ID_TOOL :: init --> hopper: (%d, %f, %f, %f, %f, %f, %s)",
						position[area][pos][state].direct_mode,
						position[area][pos][state].x,
						position[area][pos][state].y,
						position[area][pos][state].z,
						position[area][pos][state].p,
						position[area][pos][state].r,
						position[area][pos][state].cmd.c_str());
					state++;
				}
			}
		}
	}	//*/
/*	for (int area = LEFT; area <= RIGHT; area++) {
		for (int pos = 0; pos < 13; pos++) {
			getline(fin, buffer);
			if (buffer[0] == '#' || buffer.size() == 0) {
				pos--;
				continue;
			}
			std::stringstream ss;
			ss << buffer;
			float x, y, z, p, r;
			ss >> x >> y >> z >> p >> r;
			position[FIND_TOOL][area][pos].direct_mode = true;
			position[FIND_TOOL][area][pos].x = x;
			position[FIND_TOOL][area][pos].y = y;
			position[FIND_TOOL][area][pos].z = z;
			position[FIND_TOOL][area][pos].p = p;
			position[FIND_TOOL][area][pos].r = r;
			position[FIND_TOOL][area][pos].cmd = "id_tool: FIND_TOOL";
			ss.str("");
		}
	}
	for (int area = LEFT; area <= RIGHT; area++) {
		for (int pos = 0; pos < 5; pos++) {
			getline(fin, buffer);
			if (buffer[0] == '#' || buffer.size() == 0) {
				pos--;
				continue;
			}
			std::stringstream ss;
			ss << buffer;
			float x, y, z, p, r;
			ss >> x >> y >> z >> p >> r;
			position[FIND_TOP][area][pos].direct_mode = true;
			position[FIND_TOP][area][pos].x = x;
			position[FIND_TOP][area][pos].y = y;
			position[FIND_TOP][area][pos].z = z;
			position[FIND_TOP][area][pos].p = p;
			position[FIND_TOP][area][pos].r = r;
			position[FIND_TOP][area][pos].cmd = "id_tool: FIND_TOP";
			ss.str("");
		}
	}	//*/

	// open first camera device found in system
	// according to OpenCV documentation, this can accept a filename.
	//    so far, that hasn't worked.
	VideoCapture capture(0);
	if (!capture.isOpened()) {
		ROS_ERROR("unable to open default camera device (0) - fatal - bailing.");
		return(30);
	}

	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<wesley::arm_point>("/arm/put/point", 1000);
	ros::Subscriber sub = nh.subscribe("/arm/response", 1000, &block_arm_wait);

	while(pub.getNumSubscribers() <= 0) {
		// sit and wait until our publisher has a subscriber to listen to.
	}
	ROS_INFO("ID_TOOL --> number of subscribers; (%d)", pub.getNumSubscribers());


	// create a window and the matrices we'll need to job_state.
	// the window is for testing and lets the coder see what
	//    the camera is seeing / looking at.
DBGCV	namedWindow("frame", CV_WINDOW_AUTOSIZE);
	Mat frame, thresh, viewport, swallow;

	// vectors for the findContours, approxPolyDP and HoughCircles
	//    keep the creation of these vectors outside of the loop so
	//    that they are neither created nor destroyed each iteration.
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	vector<Point> approx;
	vector<Vec3f> circles;

	// FIND_DISTANCE stuff. moments to find the center of the contour.
	Moments mu;		// contour moments
	Point2f mc;		// contours mass center
	// allows shapes to be drawn in the frame with different colors
	//    for respective shapes and stages.
	// for debugging only - useless when wesley is headless.
	Scalar color = CV_RGB(0, 0, 0);

	// keep track of having found what we're looking for. effectively,
	//    when both of these are true, we've found what we're looking for.
	bool tool_found = false;
	bool top_found = false;
	// overall catch - if we don't find any tools at all after our many loops!
	//    also allows an easy out in case of huge failure.
	bool failure = false;

	// HUA means "everything but no". this is actually our finishing statement.
	//    when this is true the tool is held and in place.
	bool HUA = false;

	// keep track of which position was last looked at - let's us start
	//    from where we left off and not have to start all the way over.
	size_t area = LEFT;		// via enum(tool_area), LEFT = 0, CENTER = 1, and RIGHT = 2
	size_t pos = 0;
	int contour_idx = -1;
	
	const unsigned char frame_offset_y = 0;

	wesley::arm_point pickup;
	double z_dist = 0.0f;

	#define EVER ;;
	for(EVER) {
		ROS_WARN("ID_TOOL :: switch(job_state) [%d]", job_state);
		switch(job_state) {
			case FIND_TOOL: {
				for(; pos < MAX_ID_TOOL; pos++) {
					for(; area <= RIGHT; area++) {
						ROS_INFO("ID_TOOL :: (FIND_TOOL) --> switching state to find tool");
						ROS_INFO("ID_TOOL :: (FIND_TOOL) --> for(%d:%d)", area, pos);
						// move arm to position[job_state][area][trial];
						ROS_INFO("ID_TOOL :: (FIND_TOOL) --> going to: (%f, %f, %f, %f, %f)",
									position[area][pos][job_state].x,
									position[area][pos][job_state].y,
									position[area][pos][job_state].z,
									position[area][pos][job_state].p,
									position[area][pos][job_state].r);
						pub.publish(position[area][pos][job_state]);
						wait_on_arm();
						ROS_INFO("ID_TOOL :: (FIND_TOOL) --> released from block_arm_wait");
						// wait for return response from arm indicating finshed moving.
						// acquire a frame to job_state
						capture >> frame;
						// swallow the next 6 frames from the buffer.
						// this was discovered to be a problem from the camera buffer.
						// for some reason the camera buffer stores 6 frames before
						//    catching up to where we want it to be. 6 is the minimum.
						//    anything more is also acceptable; the frame capture is
						//    rather fast.
						for (int i = 0; i < 6; i++) {
							capture >> swallow;
						}
						// create a rectangle for our region of interest
						Rect ROI_tool = Rect(160, 80, 300, 360);
						// draw a red outline around the region of interest
//						rectangle(frame, ROI_tool, CV_RGB(0xF8, 0x00, 0x12), 1);

						// apply ROI_tool to frame and copy that section into a new matrix
						Mat viewport = frame(ROI_tool);
						contour_idx = process_frame(viewport, thresh, contours);
						double good_area = 25000;
						double contour_area = contourArea(contours[contour_idx], false);
						ROS_INFO("ID_TOOL :: FIND_TOOL ---------------------> area: (%f)", contour_area);
					//	if (contour_idx > -1 && (contour_area > good_area)) {
						if (contour_idx > -1) {
							approxPolyDP(contours[contour_idx],
										 approx,
										 arcLength(contours[contour_idx], true) * .015,
										 true);
							switch(tool) {
								case SQUARE:
									// these constants that approx.size() are checked 
									//    against were found after many, many tests.
									// NO TOUCHY!~
									if (approx.size() == 4 ||
										approx.size() == 5 ||
										approx.size() == 6 ||
										approx.size() == 7) {
										ROS_WARN("match(SQUARE)");
										// skyblue
DBGCV										color = CV_RGB(0x87, 0xCE, 0xEB);
										tool_found = true;
										job_state = FIND_TOP;
									} else {
										tool_found = false;
DBGCV										color = CV_RGB(0xFF, 0x22, 0x44);
									}
								break;
								case TRIANGLE:
									if (approx.size() == 3 ||
										approx.size() == 4 ||
										approx.size() == 5) {
										ROS_WARN("match(TRIANGLE)");
										// tomato
DBGCV										color = CV_RGB(0xFF, 0x63, 0x47);
										tool_found = true;
										job_state = FIND_TOP;
									} else {
										tool_found = false;
DBGCV										color = CV_RGB(0xFF, 0x22, 0x44);
									}
								break;
								case CIRCLE:
									if (approx.size() == 8 ||
										approx.size() == 9 ||
										approx.size() == 10) {
										ROS_WARN("match(CIRCLE)");
										// teal
DBGCV										color = CV_RGB(0x00, 0x80, 0x80);
										tool_found = true;
										job_state = FIND_TOP;
									} else {
										tool_found = false;
DBGCV										color = CV_RGB(0xFF, 0x22, 0x44);
									}
								break;
								default:

								break;
							}	// end switch(tool) for FIND_TOOL
							if (tool_found == true) {
								ROS_INFO("ID_TOOL :: (FIND_TOOL) --> found a tool.");
DBGCV								polylines(viewport, approx, true, color);
								job_state = FIND_TOP;
								break;		// from for(area);
							} else {
								// moste likely, approx.size() did not match our criteria
//DBGCV								imshow("frame", frame);
//DBGCV								while(waitKey() != 27);
								// didn't find what we were looking for,
								// try next position.
								continue;	// with for(area)
							}
DBGCV							imshow("frame", frame);
DBGCV							while(waitKey() != 27);
						} else {
							ROS_INFO("ID_TOOL :: (FIND_TOOL) :: process frame --> contour_idx reports -1");
							// contour_idx was -1, didn't find a contour. try
							//    next position.
							tool_found = false;
							continue;
						}	// end if(contour_idx)
					}	// end for(area) within each trial
						ROS_INFO("ID_TOOL :: (FIND_TOOL) :: for(trial) done --> for(%d", pos);
						ROS_INFO("ID_TOOL :: (FIND_TOOL) --> ending reason(%d)", tool_found);
					if (tool_found == true) {
//						destroyWindow("frame");
						ROS_INFO("ID_TOOL :: (FIND_TOOL) :: found a tool.");
						job_state = FIND_TOP;
						break;
					} else {
//						destroyWindow("frame");
						tool_found = false;
						// didn't find a tool this go-round
						// reset area and start again.
						area = LEFT;
					}
				}	// end for(trial)
				if (tool_found == false) {
					// after checking all of our positions, no tool was found.
					// inidicate failure so that program will halt cleanly.
					failure = true;
				}
			}	// end case(FIND_TOOL);
			break;
			case FIND_TOP: {
				ROS_INFO("ID_TOOL :: (FIND_TOP) :: --> switching state to find top of tool");
				// move arm to position[job_state][area][pos];
				pub.publish(position[area][pos][job_state]);
				wait_on_arm();
				//    job_state is the main logic handler/tracker
				//    area follows from the for loop in FIND_TOOL
				capture >> frame;
				for (int i = 0; i < 6; i++) {
					capture >> swallow;
				}
				// lose the upper 100 pixels of the frame to avoid catching the edge
				//    of the board and all the nothingness beyond.
				Rect ROI_tool = Rect(0, frame_offset_y, 640, (480 - frame_offset_y));
DBGCV				rectangle(frame, ROI_tool, CV_RGB(0xD0, 0x00, 0x6E), 1);
				Mat viewport = frame(ROI_tool);
				contour_idx = process_frame(viewport, thresh, contours);
				double good_area = 30000;
				double contour_area = contourArea(contours[contour_idx], false);
				ROS_INFO("ID_TOOL :: FIND_TOP -----------------------> area: (%f)", contour_area);
				if (contour_idx > -1 && (contour_area > good_area)) {
			//	if (contour_idx > -1) {
					approxPolyDP(contours[contour_idx],
								 approx,
								 arcLength(contours[contour_idx], true) * .015,
								 true);
					switch(tool) {
						case SQUARE:
							if (approx.size() == 4) {
								// goldenrod
DBGCV								color = CV_RGB(0xDA, 0xA5, 0x20);
								ROS_WARN("match_TOP(SQUARE)");
								top_found = true;
							}
						break;
						case TRIANGLE:
							if (approx.size() == 3) {
								// lightpink
DBGCV								color = CV_RGB(0xFF, 0xB6, 0xC1);
								ROS_WARN("match_TOP(TRIANGLE)");
								top_found = true;
							} else {
								{
									// somtimes a triangle can pick up the shadow as
									//    part of the contour. here we'll run the
									//    approximated polygon through another run
									//    of approxPolyDP and use a large value to
									//    clear off any of the staggler parts.
									// 123 was picked through testing as a fairly sane
									//    value. anything between 50 and 300 should
									//    work just as well.
									// additionally, the shade within this range should
									//    be small enough to not throw off the top
									//    finding code in FIND_TOP. this means we can
									//    still use the original approx vector.
									vector<Point> secapprox;
									approxPolyDP(approx, secapprox, 123, true);
									if (secapprox.size() == 3) {
DBGCV										color = CV_RGB(0xFF, 0xB6, 0xC1);
										ROS_WARN("match_TOP(TRIANGLE--2)");
										top_found = true;
									}
								}
							}
						break;
						case CIRCLE:
							if (approx.size() == 8) {
								// olive
DBGCV								color = CV_RGB(0x80, 0x80, 0x00);
								ROS_WARN("match_TOP(CIRCLE)");
								top_found = true;
							}
						break;
						default:
						break;
					}	// end switch(tool) for FIND_TOP
					if (top_found == true) {
					ROS_INFO("ID_TOOL :: (FIND_TOP) :: for(pos) --> found the top. not finished with loop yet.");
						job_state = FIND_DISTANCE;
//						break;
					} else {
						// go to the next position.
//						continue;
					}
DBGCV						polylines(viewport, approx, true, color);
				}
				if (top_found == true) {
					ROS_INFO("ID_TOOL :: (FIND_TOP) :: for(pos) --> found the top. tool is double-confirmed.");
					job_state = FIND_DISTANCE;
					//	break;
				} else {
					ROS_INFO("ID_TOOL :: (FIND_TOP) --> DID NOT FIND TOP. no confidence, restart trials.");
					if (area == RIGHT && pos == (MAX_ID_TOOL - 1)) {
						ROS_INFO("ID_TOOL :: (FIND_TOP) --> reached end. next trial would over-run. bail.");
						failure = true;
					}
					else if (area == RIGHT && pos < MAX_ID_TOOL) {
						area = LEFT;
						pos += 1;
					}
					else {
					area += 1;
					}
						
					job_state = FIND_TOOL;
				}
DBGCV				imshow("frame", frame);
DBGCV				while(waitKey() != 27);
			}	// end case(FIND_TOP);
			break;
			// FIND_CENTER and FIND_DISTANCE should be the same job_state.
			case FIND_DISTANCE: {
				ROS_INFO("ID_TOOL :: FIND_DISTANCE --> switching state to find relation to tool");
				// use the contour from above (FIND_TOP) and use it
				//    to judge tool distance from camera
				// this should work since contours is filled inside the loop
				//    and we're not touching it this go-round.
				// if we're in FIND_DISTANCE, this should always succeed.
				
				// these are the actual lengths of diagonals of the tools in millimeters.
				const double tool_d[4] = {
					0.0f,
					72.4,		// SQUARE diagonal
					57.15,		// TRIANGLE edge
					50.8, 		// CIRCLE diameter
				};
				
//				wesley::arm_point t = position[area][pos][FIND_TOP];	// t, for temp;
				wesley::arm_point camera = position[area][pos][FIND_TOP];
				ROS_INFO("ID_TOOL :: (FIND_TOOL) --> tip: (%f, %f, %f, %f, %f)",
							camera.x,
							camera.y,
							camera.z,
							camera.p,
							camera.r);
				// alpha: the angle of the arm in its own reference frame
				double alpha_r = atan2(camera.y, camera.x);
				// camera(x, y, z) is the point of the camera in the robot's frame.
				camera.x += (27*(cos(alpha_r)) + 47*(sin(alpha_r)));
				camera.y -= (47*(cos(alpha_r)) - 27*(sin(alpha_r)));
				double tool_area = contourArea(contours[contour_idx], false);
				ROS_INFO("ID_TOOL :: (FIND_TOOL) --> camera: (%f, %f, %f, %f, %f)",
							camera.x,
							camera.y,
							camera.z,
							camera.p,
							camera.r);
				z_dist = distance_to_tool(tool_area, tool);
				ROS_INFO("ID_TOOL :: FIND_DISTANCE --> z_dist: %f", z_dist);
				mu = moments(contours[contour_idx], false);
				mc = Point2f((mu.m10 / mu.m00),
							 (mu.m01 / mu.m00));
				Point2f offset(
					  mc.x - 320,	// x coordinate offset from center of camera frame
					-(mc.y - ((480 - frame_offset_y) / 2))	
									// same, but for y. here, camera.y moves opposite
				);					//    the frame of the robot.
				float roll_x = 0.0f;
				float roll_y = 0.0f;
				float roll_r = 0.0f;
				float roll_d = 0.0f;
				float ratio= 0.0f;
				double hypot = 0.0f;
				switch(tool) {
					case SQUARE: {
						// find the longest side in an attempt to pick up the tool in the same manner
						short ha, hb;
						if (( sqrt(pow((approx[0].x - approx[1].x), 2) + pow((approx[0].y - approx[1].y), 2)) ) >
						    ( sqrt(pow((approx[1].x - approx[2].x), 2) + pow((approx[1].y - approx[2].y), 2)) ) ) {
							ha = 1;
							hb = 0;
						} else {
							ha = 2;
							hb = 1;
						}
						// calculate the angle offset of the long side.
						roll_x = approx[ha].x - approx[hb].x;
						roll_y = approx[ha].y - approx[hb].y;
						roll_r = atan2(roll_y, roll_x);
						roll_d = 90 - (roll_r * 180 / 3.14159);
						ROS_INFO("ID_TOOL :: FIND_DISTANCE --> offset degree of long edge: %f", roll_d);

						hypot = sqrt(pow((approx[hb].x - approx[ha + 1].x), 2)
								   + pow((approx[hb].y - approx[ha + 1].y), 2));

					}
					break;
					case TRIANGLE:
						// triangle work here
						roll_x = approx[1].x - approx[0].x;
						roll_y = approx[1].y - approx[0].y;

						hypot = sqrt(pow(roll_x, 2)
								   + pow(roll_y, 2));
						
						roll_r = atan2(roll_y, roll_x);
						roll_d = 90 - (roll_r * 180 / 3.14159);

						break;
					case CIRCLE:
						// circle work here
						hypot = sqrt(pow((approx[4].x - approx[0].x), 2)
								   + pow((approx[4].y - approx[0].x), 2));
						break;
					default:
						ROS_WARN("ID_TOOL :: FIND_DISTANCE --> switch(tool): default, shouldn't be here. continuing.");
						// like the warning says -- shouldn't be here, don't know what else to do,
						//    and so just going to continue with the loops and let the program 
						//    sort itself out.
						break;
				}

				ratio = tool_d[tool] / hypot;

				ROS_WARN("ID_TOOL :: FIND_DISTANCE --> offset(xp, yp): offset(%f, %f)", offset.x, offset.y);
				// translate the pixel offset into millimeters (the unit of the robot's frame)
				offset.x *= ratio;
				offset.y *= ratio;
				ROS_WARN("ID_TOOL :: FIND_DISTANCE --> offset(xm, ym): offset(%f, %f)", offset.x, offset.y);
				// theta is the angle of the line between camera center and tool center
				//    in relation to the camera's frame.
				double theta_r = (atan2(offset.y, offset.x));
				double lambda_r = (M_PI_2 - alpha_r + theta_r);
				// store the original values so that we can manipulate them both at the same time
				float xc = offset.x;
				float yc = offset.y;
				// translate the camera->tool offset into the robot's frame.
				offset.x =  ((xc * cos(lambda_r)) + (yc * sin(lambda_r)));
				// flipped from camera's negative y to robot's positive y.
				offset.y = -((yc * cos(lambda_r)) - (xc * sin(lambda_r)));

				ROS_INFO("ID_TOOL :: FIND_DISTANCE --> opening hand");
				pickup.direct_mode = false;
				pickup.cmd = "release";
				pub.publish(pickup);
				wait_on_arm();

				pickup.direct_mode = true;
				pickup.x = camera.x - offset.x;
				pickup.x -= (10*sin(roll_r + alpha_r));
				pickup.y = camera.y + offset.y;
				pickup.y += (10*cos(roll_r + alpha_r));
				pickup.z = camera.z;				// left alone for now. adjusted in a few lines.
				pickup.p = camera.p;
				pickup.r = fmod((camera.r - roll_d), 180.0);
				pickup.cmd = "id_tool: pickup";
				ROS_WARN("ID_TOOL :: FIND_DISTANCE --> pickup (%f, %f, %f (-> %f), %f, %f)",
					pickup.x,
					pickup.y,
					pickup.z, (pickup.z - z_dist),
					pickup.p,
					pickup.r);
				ROS_INFO("ID_TOOL :: FIND_DISTANCE --> pickup point found. publishing.");
DBGCV				std::cout << "ID_TOOL :: FIND_DISTANCE --> point: " << pickup << std::endl;

				// light purple
DBGCV				color = CV_RGB(0xAD, 0x66, 0xD5);
DBGCV				circle(frame, mc, 5, color, CV_FILLED);

DBGCV				imshow("frame", frame);
DBGCV				while(waitKey() != 27);
				// write all of this information to a file.. or, perhaps
				//    to an ROS service that translates our co-ordinates
				//    into appropriate movement codes to the arm.
				//
				// 08MAR14 - emg
				// the current idea is instead to have this code pick the
				//    tool up and return a 1 to the caller that we have
				//    succeeded in collecting the tool.
				// that's probably the way to go.
				job_state = TOOL_GRASP;
			}
			break;
			case TOOL_GRASP:
				ROS_INFO("ID_TOOL :: TOOL_GRASP --> preparing to pick up tool.");
				pub.publish(pickup);
				wait_on_arm();
				ROS_WARN("ID_TOOL :: TOOL_GRASP --> first offset published.");

				ROS_INFO("ID_TOOL :: TOOL_GRASP --> midline point.");
				pickup.z -= (z_dist - 50);
				pub.publish(pickup);
				wait_on_arm();

//				wait_on_arm();

				sleep(1);		// sleep an extra second beyond what &block_arm_wait() does.
				pickup.z = (position[area][pos][FIND_TOP].z - z_dist);
				pub.publish(pickup);
				wait_on_arm();
				ROS_WARN("ID_TOOL :: TOOL_GRASP --> second offset published.");

				
				ROS_INFO("ID_TOOL :: TOOL_GRASP --> preparing to grasp");
				ROS_INFO("ID_TOOL :: TOOL_GRASP --> switching state to gather tool.");
				pickup.direct_mode = false;
				{
					std::stringstream ss;
					ss.str("");
					ss << "grasp " << tool;
					pickup.cmd = ss.str();
				}
				pub.publish(pickup);
				wait_on_arm();
				ROS_INFO("ID_TOOL :: TOOL_GRASP --> tool should be had.");

				// lift up.
				pickup.direct_mode = true;
				pickup.z = position[area][pos][FIND_TOP].z;
				pickup.p = -75;
				pickup.cmd = "lifting straight";
				ROS_INFO("ID_TOOL :: (FIND_TOOL) --> going to: (%f, %f, %f, %f, %f)",
							position[area][pos][job_state].x,
							position[area][pos][job_state].y,
							position[area][pos][job_state].z,
							position[area][pos][job_state].p,
							position[area][pos][job_state].r);
				pub.publish(pickup);
				wait_on_arm();	

				pickup.direct_mode = false;
				pickup.cmd = "carry";
				pub.publish(pickup);
				wait_on_arm();
				ROS_INFO("ID_TOOL :: TOOL_GRASP --> tool in possession and carried. exiting.");

// i think this is 				
				HUA = true;
			break;				
			default:
				// no known cases that bring us here.
			break;
		}	// end switch(job_state)
//DBGCV		imshow("frame", frame);
//DBGCV		while(waitKey() != 27);
//		if (tool_found == true && top_found == true) {
//			ROS_INFO("ID_TOOL(for(EVER)) --> CONFIRMED! finishing out.");
//			ROS_INFO("ID_TOOL(FIND_TOOL) :: found tool at: (%f, %f, %f, %f, %f)",
//						position[FIND_TOP][area][pos].x,
//						position[FIND_TOP][area][pos].y,
//						position[FIND_TOP][area][pos].z,
//						position[FIND_TOP][area][pos].p,
//						position[FIND_TOP][area][pos].r);
//			break;
//		}
		if (HUA == true) {
			ROS_INFO("ID_TOOL :: HUA?");
			break;
		}
		if (failure == true) {
			ROS_INFO("ID_TOOL :: (for(EVER)) --> couldn't find a tool. bailing.");
			// we couldn't find anything. THIS IS FATAL TO CONTINUED OPERATION.
			//
			// 08MAR14 - emg
			// current job_state decides that if we fail, we'll do our best to
			//    pick up a random tool. 33% chance of getting the right tool
			//    is better than a 0% chance because we gave up.
			break;
		}
	}	// end for(EVER);

	// well - we're here. at the end of it all.
	// clean up as needed. c++ destructors will be called automatically.
	// return true or false (1 or 0) indicating that we found the tool,
	//    grasped it, picked it up, and put it in carry.
	
	return(HUA);
}
