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
#define EMGDBG (1)
#define DBGOUT if EMGDBG std::cout

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
	if (argc != 2) {
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
	string file_position_tool = "./position_tool.lst";
	if (stat(file_position_tool.c_str(), &verify) == 0) {
		fin.open(file_position_tool.c_str(), std::ifstream::in);
	} else {
		ROS_ERROR("ID_TOOL --> unable to locate %s - fatal; bailing.", file_position_tool.c_str());
		return(60);
	}

	const short MAX_ID_TOOL = 13;

	wesley::arm_point position[2][3][MAX_ID_TOOL];
	std::string buffer;
//	struct coordinate position[2][3][13];
	for (int area = LEFT; area <= RIGHT; area++) {
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
	}

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
	namedWindow("frame", CV_WINDOW_AUTOSIZE);
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
	size_t trial = 0;
	size_t area = LEFT;		// via enum(tool_area), LEFT = 0, CENTER = 1, and RIGHT = 2
	size_t pos = 0;
	int contour_idx = -1;

	wesley::arm_point pickup;
	#define EVER ;;
	for(EVER) {
		ROS_WARN("ID_TOOL :: switch(job_state) [%d]", job_state);
		switch(job_state) {
			case FIND_TOOL: {
				for(; trial < MAX_ID_TOOL; trial++) {
					for(; area <= RIGHT; area++) {
						ROS_INFO("ID_TOOL :: (FIND_TOOL) --> for(%d:%d)", trial, area);
						// move arm to position[job_state][area][trial];
						waiting = true;
						ROS_INFO("ID_TOOL :: (FIND_TOOL) --> going to: (%f, %f, %f, %f, %f)",
									position[job_state][area][pos].x,
									position[job_state][area][pos].y,
									position[job_state][area][pos].z,
									position[job_state][area][pos].p,
									position[job_state][area][pos].r);
						pub.publish(position[job_state][area][pos]);
						do {
							ros::spinOnce();
						} while(waiting);
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
						rectangle(frame, ROI_tool, CV_RGB(0xF8, 0x00, 0x12), 1);

						// apply ROI_tool to frame and copy that section into a new matrix
						Mat viewport = frame(ROI_tool);
						contour_idx = process_frame(viewport, thresh, contours);
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
										color = CV_RGB(0x87, 0xCE, 0xEB);
										tool_found = true;
										job_state = FIND_TOP;
									} else {
										tool_found = false;
										color = CV_RGB(0xFF, 0x22, 0x44);
									}
								break;
								case TRIANGLE:
									if (approx.size() == 3 ||
										approx.size() == 4 ||
										approx.size() == 5) {
										ROS_WARN("match(TRIANGLE)");
										// tomato
										color = CV_RGB(0xFF, 0x63, 0x47);
										tool_found = true;
										job_state = FIND_TOP;
									} else {
										tool_found = false;
										color = CV_RGB(0xFF, 0x22, 0x44);
									}
								break;
								case CIRCLE:
									if (approx.size() == 8 ||
										approx.size() == 9 ||
										approx.size() == 10) {
										ROS_WARN("match(CIRCLE)");
										// teal
										color = CV_RGB(0x00, 0x80, 0x80);
										tool_found = true;
										job_state = FIND_TOP;
									} else {
										tool_found = false;
										color = CV_RGB(0xFF, 0x22, 0x44);
									}
								break;
								default:

								break;
							}	// end switch(tool) for FIND_TOOL
							if (tool_found == true) {
								ROS_INFO("ID_TOOL :: (FIND_TOOL) --> found a tool.");
								imshow("frame", frame);
								while(waitKey() != 27);
								job_state = FIND_TOP;
								break;		// from for(area);
							} else {
								imshow("frame", frame);
								while(waitKey() != 27);
								// didn't find what we were looking for,
								// try next position.
								continue;	// with for(area)
							}
						} else {
							ROS_INFO("ID_TOOL :: (FIND_TOOL) :: process frame --> contour_idx reports -1");
							// contour_idx was -1, didn't find a contour. try
							//    next position.
							tool_found = false;
							continue;
						}	// end if(contour_idx)
					}	// end for(area) within each trial
						ROS_INFO("ID_TOOL :: (FIND_TOOL) :: for(trial) done --> for(%d", trial);
						ROS_INFO("ID_TOOL :: (FIND_TOOL) --> ending reason(%d)", tool_found);
					if (tool_found == true) {
						destroyWindow("frame");
						ROS_INFO("ID_TOOL :: (FIND_TOOL) :: found a tool.");
						job_state = FIND_TOP;
						break;
					} else {
						destroyWindow("frame");
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
				for(pos = 0; pos < 5; pos++) {
					ROS_INFO("ID_TOOL :: (FIND_TOP) :: for(pos) --> for(%d)", pos);
					// move arm to position[job_state][area][pos];
					waiting = true;
					pub.publish(position[job_state][area][pos]);
					do {
						ros::spinOnce();
					} while(waiting);
					//    job_state is the main logic handler/tracker
					//    area follows from the for loop in FIND_TOOL
					capture >> frame;
					for (int i = 0; i < 6; i++) {
						capture >> swallow;
					}
					contour_idx = process_frame(frame, thresh, contours);
					if (contour_idx > -1) {
						approxPolyDP(contours[contour_idx],
									 approx,
									 arcLength(contours[contour_idx], true) * .015,
									 true);
						switch(tool) {
							case SQUARE:
								if (approx.size() == 4) {
									// goldenrod
									color = CV_RGB(0xDA, 0xA5, 0x20);
									ROS_WARN("match_TOP(SQUARE)");
									top_found = true;
								}
							break;
							case TRIANGLE:
								if (approx.size() == 3) {
									// lightpink
									color = CV_RGB(0xFF, 0xB6, 0xC1);
									ROS_WARN("match_TOP(TRIANGLE)");
									top_found = true;
								}
							break;
							case CIRCLE:
								if (approx.size() == 8) {
									// olive
									color = CV_RGB(0x80, 0x80, 0x00);
									ROS_WARN("match_TOP(CIRCLE)");
									top_found = true;
								}
							break;
							default:
							break;
						}	// end switch(tool) for FIND_TOP
						if (top_found == true) {
						ROS_INFO("ID_TOOL :: (FIND_TOP) :: for(pos) --> found the top. not finished with loop yet.");
							imshow("frame", frame);
							while(waitKey() != 27);
							job_state = FIND_DISTANCE;
							break;
						} else {
							imshow("frame", frame);
							while(waitKey() != 27);
							// go to the next position.
							continue;
						}
					} else {
						imshow("frame", frame);
						while(waitKey() != 27);
						// contour_idx was -1. no contour found.
						// go to next position.
						continue;
					}
				}	// end for(pos)
				if (top_found == true) {
					ROS_INFO("ID_TOOL :: (FIND_TOP) :: for(pos) --> found the top. tool is double-confirmed.");
					job_state = FIND_DISTANCE;

				//	break;
				} else {
						ROS_INFO("ID_TOOL :: (FIND_TOP) --> DID NOT FIND TOP. no confidence, restart trials.");
					// didn't find the shape 
					// go back to FIND_TOOL from where we left off + 1;
					if (area == RIGHT && trial == 4) {
						ROS_INFO("ID_TOOL :: (FIND_TOP) --> reached the end. next trial would overrun. bail.");
						failure = true;
					}
					else if (area == RIGHT && trial < 4) {
						area = LEFT;
						trial += 1;
					}
					else {
						area += 1;
					}
			

					job_state = FIND_TOOL;
				}
			}	// end case(FIND_TOP);
			break;
			// FIND_CENTER and FIND_DISTANCE should be the same job_state.
			case FIND_DISTANCE: {
				// use the contour from above (FIND_TOP) and use it
				//    to judge tool distance from camera
				// this should work since contours is filled inside the loop
				//    and we're not touching it this go-round.
				// if we're in FIND_DISTANCE, this should always succeed.
				
				const double tool_d[4] = {
					0.0f,
					72.4,		// SQUARE diagonal
					57.15,		// TRIANGLE edge
					50.8, 		// CIRCLE diameter
				};

				wesley::arm_point t = position[FIND_TOP][area][pos];	// t, for temp;
				double tool_area = contourArea(contours[contour_idx], false);
				double z_dist = distance_to_tool(tool_area, tool);
				ROS_INFO("ID_TOOL :: FIND_DISTANCE --> z_dist: %f", z_dist);
				mu = moments(contours[contour_idx], false);
				mc = Point2f((mu.m10 / mu.m00),
							 (mu.m01 / mu.m00));
				Point2f offset(
					  mc.x - 320,	// x coordinate offset from center of camera frame
					-(mc.y - 240)	// same, but for y. here, camera.y moves opposite
				);					//    the frame of the robot.
				float off_x, off_y;
				float off_r, off_d;
				float ratio= 0.0f;
				double hypot = 0.0f;
				switch(tool) {
					case SQUARE: {
						short ha, hb;
						if (( sqrt(pow((approx[0].x - approx[1].x), 2) + pow((approx[0].y - approx[1].y), 2)) ) >
						    ( sqrt(pow((approx[1].x - approx[2].x), 2) + pow((approx[1].y - approx[2].y), 2)) ) ) {
							ha = 1;
							hb = 0;
						} else {
							ha = 2;
							hb = 1;
						}
						off_x = approx[ha].x - approx[hb].x;
						off_y = approx[ha].y - approx[hb].y;
						off_r = atan2(off_y, off_x);
						off_d = 90 - (off_r * 180 / 3.14159);

						hypot = sqrt(pow((approx[hb].x - approx[ha + 1].x), 2)
								   + pow((approx[hb].y - approx[ha + 1].y), 2));

					}
					break;
					case TRIANGLE:
						// triangle work here
						off_x = approx[1].x - approx[0].x;
						off_y = approx[1].y - approx[0].y;

						hypot = sqrt(pow(off_x, 2)
								   + pow(off_y, 2));
						
						off_r = atan2(off_y, off_x);
						off_d = 90 - (off_r * 180 / 3.14159);

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

				offset.x *= ratio;
				offset.y *= ratio;
				offset.x += 47 + (20 * cos((off_d * 3.14159 / 180.0)));
				offset.y += 47 + (20 * sin((off_d * 3.14159 / 180.0)));

				pickup.direct_mode = true;
				pickup.x = t.x + offset.x;
				pickup.y = t.y + offset.y;
				pickup.z = t.z;				// left alone for now. adjusted in a few lines.
				pickup.p = t.p;
				pickup.r = t.r - off_d;
				pickup.cmd = "id_tool: pickup";
				ROS_INFO("ID_TOOL :: FIND_DISTANCE --> pickup point found. publishing.");

				pub.publish(pickup);
				waiting = true;
				do {
					ros::spinOnce();
				} while(waiting);
				ROS_WARN("ID_TOOL :: FIND_DISTANCE --> first offset published.");

				sleep(1);		// sleep an extra second beyond what &block_arm_wait() does.
				pickup.z = t.z - z_dist - 25;
				pub.publish(pickup);
				waiting = true;
				do { 
					ros::spinOnce();
				} while(waiting);
				ROS_WARN("ID_TOOL :: FIND_DISTANCE --> second offset published.");

				job_state = TOOL_GRASP;
				
				// light purple
				color = CV_RGB(0xAD, 0x66, 0xD5);
				circle(frame,
					   mc,
					   5,
					   color,
					   CV_FILLED);

				imshow("frame", frame);
				while(waitKey() != 27);
				// write all of this information to a file.. or, perhaps
				//    to an ROS service that translates our co-ordinates
				//    into appropriate movement codes to the arm.
				//
				// 08MAR14 - emg
				// the current idea is instead to have this code pick the
				//    tool up and return a 1 to the caller that we have
				//    succeeded in collecting the tool.
				// that's probably the way to go.
			}
			break;
			case TOOL_GRASP:
				pickup.direct_mode = false;
				pickup.cmd = "grasp";
				pub.publish(pickup);
				waiting = true;
				do {
					ros::spinOnce();
				} while(waiting);
				ROS_INFO("ID_TOOL :: TOOL_GRASP --> tool should be had.");

				pickup.cmd = "carry";
				waiting = true;
				do {
					ros::spinOnce();
				} while(waiting);
				ROS_INFO("ID_TOOL :: TOOL_GRASP --> tool in possession and carried. exiting.");

				HUA = true;
			break;				
			default:
				// no known cases that bring us here.
			break;
		}	// end switch(job_state)
		imshow("frame", frame);
		while(waitKey() != 27);
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
			ROS_INFO("ID_TOOL :: HUA!");
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
