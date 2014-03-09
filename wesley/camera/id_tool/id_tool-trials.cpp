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
 * 	switch(process) {
 * 		case (find_tool) {
 * 			for(pos = 1 - 5) {
 * 				for(tool = 1 - 3) {
 * 					position arm at position_tool[pos][tool]
 * 					take a picture
 * 					process photo
 * 					if requested tool is found
 * 						set process to find_top
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
 * 				process photo
 * 				if top found
 * 					set process to id_center
 * 				else
 * 					continue
 * 			}
 * 			if this point is reached, then we did not find top
 * 			set process to find_tool
 * 			continue
 * 		}
 * 		case (id_center) {
 * 			use same frame as captureed in find_top
 * 			reprocess to find the center
 * 			find the distance to the center
 * 			translate between co-ordinate frames
 * 			set process to grab_tool
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

#include <iostream>		// basic I/O

// personal debug defines. set to (0) to turn off.
#define EMGDBG (1)
#define DBGOUT if EMGDBG std::cout

// coordinate structure - this will be filled from a file.
// for the oil rigs - use 'coordinates_flame.lst'
// for the tools    - use 'coordinates_tools.lst'
//
// format of the file is:
//    one coordinate per line separated by a space:
//    	 x y z w
//    file does support comments: begin each line of
//       a comment with a #
struct coordinate {
	int x;
	int y;
	int z;
	coordinate(int x=0, int y=0, int z=0) : 
		x(x), y(y), z(z) { };
};

// quick help in case the program is called incorrectly.
void program_help(const char* arg_zero) {
	std::cerr << "please call " << arg_zero << " with one of:" << std::endl;
	std::cerr << "\t(s or 1), (t or 2), or (c or 3)" << std::endl;
}

// returns index of the biggest contour found within the frame;
// returns -1 if no contour was found.
int process_frame(Mat &frame, Mat &thresh, vector<vector<Point> > &contours) {
	// create a rectangle for our region of interest
	Rect ROI_tool = Rect(160, 80, 300, 360);
	// draw a red outline around the region of interest
	rectangle(frame, ROI_tool, CV_RGB(0xF8, 0x00, 0x12), 1);

	// apply ROI_tool to frame and copy that section into a new matrix
	Mat viewport = frame(ROI_tool);
	viewport.copyTo(thresh);

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
	// this program must be called with one argument (see switch below)
	if (argc != 2) {
		program_help(argv[0]);
		return(10);
	}

	// how to keep track of the tool we're looking for
	enum shapes { NONE, SQUARE, TRIANGLE, CIRCLE };
	unsigned short tool = NONE;
	// how to keep track of what section of the process we're in
	enum states { FIND_TOOL, FIND_TOP, FIND_DISTANCE };
	unsigned short process = FIND_TOOL;
	// when we're looking for the tool, this lets us keep track
	//    of which tool position we're looking at
	enum tool_area { LEFT, CENTER, RIGHT };

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
			std::cerr << "unable to understand called method. -- fatal, bailing." << std::endl;
			std::cerr << "called with: < " << argv[1][0] << " >" << std::endl << std::endl;
			program_help(argv[0]);
			return(20);
			break;
	}

	// position[process][area][pos]
	//
	// be sure to fill this with appropriate locations.
	// preferably from a file. see above the coordinate
	// structure for file definition.
	struct coordinate position[2][3][5];
	for (int process = 0; process <= 1; process++) {
		for (int area = LEFT; area <= RIGHT; area++) {
			for (int pos = 0; pos < 5; pos++) {
				position[process][area][pos] = (struct coordinate)(0, 0, 0);
			}
		}
	}

	// open first camera device found in system
	// according to OpenCV documentation, this can accept a filename.
	//    so far, that hasn't worked.
	VideoCapture capture(0);
	if (!capture.isOpened()) {
		std::cerr << "unable to open default camera device (0);" << std::endl;
		std::cerr << "fatal - bailing." << std::endl;
		return(30);
	}

	// create a window and the matrices we'll need to process.
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

	// keep track of which position was last looked at - let's us start
	//    from where we left off and not have to start all the way over.
	size_t trial = 0;
	size_t area = LEFT;		// via enum(tool_area), LEFT = 0, CENTER = 1, and RIGHT = 2
	size_t pos = 0;
	int contour_idx = -1;
	#define EVER ;;
	for(EVER) {
		switch(process) {
			case FIND_TOOL: {
				for(; trial < 5; trial++) {
					for(; area <= RIGHT; area++) {
DBGOUT << "TRIALS(FIND_TOOL) :: for(trial:area) --> for(" << trial << ":" << area << ")" << std::endl;
						// move arm to position[process][area][trial];
						// wait for return response from arm indicating finshed moving.
						// acquire a frame to process
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
						contour_idx = process_frame(frame, thresh, contours);
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
										DBGOUT << "match(SQUARE)" << std::endl;
										// skyblue
										color = CV_RGB(0x87, 0xCE, 0xEB);
										tool_found = true;
										process = FIND_TOP;
									} else {
										tool_found = false;
										color = CV_RGB(0xFF, 0x22, 0x44);
									}
								break;
								case TRIANGLE:
									if (approx.size() == 3 ||
										approx.size() == 4 ||
										approx.size() == 5) {
										DBGOUT << "match(TRIANGLE)" << std::endl;
										// tomato
										color = CV_RGB(0xFF, 0x63, 0x47);
										tool_found = true;
										process = FIND_TOP;
									} else {
										tool_found = false;
										color = CV_RGB(0xFF, 0x22, 0x44);
									}
								break;
								case CIRCLE:
									if (approx.size() == 8 ||
										approx.size() == 9 ||
										approx.size() == 10) {
										DBGOUT << "match(CIRCLE)" << std::endl;
										// teal
										color = CV_RGB(0x00, 0x80, 0x80);
										tool_found = true;
										process = FIND_TOP;
									} else {
										tool_found = false;
										color = CV_RGB(0xFF, 0x22, 0x44);
									}
								break;
								default:

								break;
							}	// end switch(tool) for FIND_TOOL
							if (tool_found == true) {
DBGOUT << "TRIALS(FIND_TOOL) :: found a tool." << std::endl;
								imshow("frame", frame);
								while(waitKey() != 27);
								process = FIND_TOP;
								break;		// from for(area);
							} else {
								imshow("frame", frame);
								while(waitKey() != 27);
								// didn't find what we were looking for,
								// try next position.
								continue;	// with for(area)
							}
						} else {
DBGOUT << "TRIALS(FIND_TOOL) :: process frame --> contour_idx reports -1" << std::endl;
							// contour_idx was -1, didn't find a contour. try
							//    next position.
							tool_found = false;
							continue;
						}	// end if(contour_idx)
					}	// end for(area) within each trial
DBGOUT << "TRIALS(FIND_TOOL) :: for(trial) done --> for(" << trial << ")" << std::endl;
DBGOUT << "TRIALS(FIND_TOOL) --> ending reason(" << tool_found << ")" << std::endl;
					if (tool_found == true) {
						destroyWindow("frame");
DBGOUT << "TRIALS(FIND_TOOL) :: found a tool." << std::endl;
						process = FIND_TOP;
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
DBGOUT << "TRIALS(FIND_TOP) :: for(pos) --> for(" << pos << ")" << std::endl;
					// move arm to position[process][area][pos];
					//    process is the main logic handler/tracker
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
									DBGOUT << "match_TOP(SQUARE)" << std::endl;
									top_found = true;
								}
							break;
							case TRIANGLE:
								if (approx.size() == 3) {
									// lightpink
									color = CV_RGB(0xFF, 0xB6, 0xC1);
									DBGOUT << "match_TOP(TRIANGLE)" << std::endl;
									top_found = true;
								}
							break;
							case CIRCLE:
								if (approx.size() == 8) {
									// olive
									color = CV_RGB(0x80, 0x80, 0x00);
									DBGOUT << "match_TOP(CIRCLE)" << std::endl;
									top_found = true;
								}
							break;
							default:
							break;
						}	// end switch(tool) for FIND_TOP
						if (top_found == true) {
DBGOUT << "TRIALS(FIND_TOP) :: for(pos) --> found the top. not finished with loop yet." << std::endl;
							imshow("frame", frame);
							while(waitKey() != 27);
							process = FIND_DISTANCE;
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
DBGOUT << "TRIALS(FIND_TOP) :: for(pos) --> found the top. tool is double-confirmed." << std::endl;
					process = FIND_DISTANCE;

					break;
				} else {
DBGOUT << "TRIALS(FIND_TOP) --> DID NOT FIND TOP. no confidence, restart trials." << std::endl;
					// didn't find the shape 
					// go back to FIND_TOOL from where we left off + 1;
					if (area == RIGHT && trial == 4) {
DBGOUT << "TRIALS(FIND_TOP) --> reached the end. next trial would overrun. bail." << std::endl;
						failure = true;
					}
					else if (area == RIGHT && trial < 4) {
						area = LEFT;
						trial += 1;
					}
					else {
						area += 1;
					}
			

					process = FIND_TOOL;
				}
			}	// end case(FIND_TOP);
			break;
			// FIND_CENTER and FIND_DISTANCE should be the same process.
			case FIND_DISTANCE: {
				// use the contour from above (FIND_TOP) and use it
				//    to judge tool distance from camera
				// this should work since contours is filled inside the loop
				//    and we're not touching it this go-round.
				// if we're in FIND_DISTANCE, this should always succeed.
				mu = moments(contours[contour_idx], false);
				mc = Point2f((mu.m10 / mu.m00),
							 (mu.m01 / mu.m00));
				// light purple
				color = CV_RGB(0xAD, 0x66, 0xD5);
				circle(viewport,
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
			default:
				// no known cases that bring us here.
			break;
		}	// end switch(process)
		imshow("frame", frame);
		while(waitKey() != 27);
		if (tool_found == true && top_found == true) {
DBGOUT << "TRIALS(for(EVER)) --> CONFIRMED! finishing out." << std::endl;
			break;
		}
		if (failure == true) {
DBGOUT << "TRIALS(for(EVER)) --> couldn't find a tool. bailing." << std::endl;
			// we couldn't find anything. THIS IS FATAL TO CONTINUED OPERATION.
			//
			// 08MAR14 - emg
			// current process decides that if we fail, we'll do our best to
			//    pick up a random tool. 33% chance of getting the right tool
			//    is better than a 0% chance because we gave up.
			break;
		}
	}	// end for(EVER);

	// well - we're here. at the end of it all.
	// clean up as needed. c++ destructors will be called automatically.
	// return true or false (1 or 0) indicating that we found the tool.
	
	return(tool_found);
}
