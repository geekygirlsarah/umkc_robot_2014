#include <cv.h>
#include <highgui.h>

using namespace cv;

#include <iostream>

Mat frame;
Mat thresh;
Mat viewport;
vector<vector<Point> > contours;
vector<Vec4i> hierarchy;
Scalar color = CV_RGB(0, 0, 0);
std::string str_shape = "pillow";

double distance_to_tool() {
}

bool find_circle(size_t ith) {
//	std::cout << "looking for: CIRCLE" << std::endl;
	vector<Point> approx;
	approxPolyDP(Mat(contours[ith]), approx, arcLength(Mat(contours[ith]), true)*.02, true);
	switch(approx.size()) {
		case 8: case 9: case 10:
			color = CV_RGB(0xDD, 0x37, 0xB4);
			str_shape = "CIRCLE";
		//	drawContours(viewport, contours, biggest_contour, \
		//				 CV_RGB(0xFF, 0x9A, 0x40), 1);
		//	putText(frame, "TRIANGLE", Point(20, 460), \
		//			FONT_HERSHEY_PLAIN, 1.5, \
		//			CV_RGB(0xFF, 0x9A, 0x40), 2);
			return(true);
			break;
		default:
			return(false);
			break;
	}
}

bool find_square(size_t ith) {
//	std::cout << "looking for: SQUARE:" << std::endl;
	vector<Point> approx;
	approxPolyDP(Mat(contours[ith]), approx, arcLength(Mat(contours[ith]), true)*.02, true);
	switch(approx.size()) {
		case 6: case 7:
			color = CV_RGB(0x4B, 0x5C, 0xD7);
			str_shape = "SQUARE";
		//	drawContours(viewport, contours, biggest_contour, \
		//				 CV_RGB(0xFF, 0x9A, 0x40), 1);
		//	putText(frame, "TRIANGLE", Point(20, 460), \
		//			FONT_HERSHEY_PLAIN, 1.5, \
		//			CV_RGB(0xFF, 0x9A, 0x40), 2);
			return(true);
			break;
		default:
			return(false);
			break;
	}
}

bool find_triangle(size_t ith) {
//	std::cout << "looking for: TRIANGLE" << std::endl;
	vector<Point> approx;
	approxPolyDP(Mat(contours[ith]), approx, arcLength(Mat(contours[ith]), true)*.02, true);
	switch(approx.size()) {
		case 3: case 4: case 5:
			color = CV_RGB(0xFF, 0x9A, 0x40);
			str_shape = "TRIANGLE";
		//	drawContours(viewport, contours, biggest_contour, \
		//				 CV_RGB(0xFF, 0x9A, 0x40), 1);
		//	putText(frame, "TRIANGLE", Point(20, 460), \
		//			FONT_HERSHEY_PLAIN, 1.5, \
		//			CV_RGB(0xFF, 0x9A, 0x40), 2);
			return(true);
			break;
		default:
			return(false);
			break;
	}
}

bool find_all(size_t ith) {

}

int main(int argc, char* argv[]) {
	if (argc != 2) {
		std::cerr << "please call " << argv[0] << " with one of:" << std::endl;
		std::cerr << "\t(t), (s), or (c)" << std::endl;
		return(10);
	}
	char looking_for = 'a';
	if (argc == 2) {
		looking_for = argv[1][0];
	}
	enum shapes { NONE, SQUARE, TRIANGLE, CIRCLE };
	short tool = NONE;

	// fancy function variable.
	bool (*call)(size_t) = NULL;
	switch(looking_for) {
		case 's':
			call = find_square;
			tool = SQUARE;
			break;
		case 't':
			call = find_triangle;
			tool = TRIANGLE;
			break;
		case 'c':
			call = find_circle;
			tool = CIRCLE;
			break;
		default:
		//	call = find_all;
		//	tool = NONE;
			return(20);
			break;
	}

	VideoCapture capture(0);
	if (!capture.isOpened()) {
		std::cerr << "unable to open camera device -- bailing" << std::endl;
		return(40);
	}

	Rect ROI_tool = Rect(160, 80, 300, 360);
	// capture frame, convert to grayscale, threshold, and flip
	bool tool_found = false;
	for(;;) {
		capture >> frame;
		rectangle(frame, ROI_tool, CV_RGB(0xF8, 0x00, 0x12), 1);

		viewport = frame(ROI_tool);
		viewport.copyTo(thresh);

		cvtColor(thresh, thresh, CV_RGB2GRAY);
		threshold(thresh, thresh, 80, 255, CV_THRESH_BINARY);
		bitwise_xor(thresh, Scalar(255, 0, 0), thresh);

	// setup to run findContours(), then display results
		findContours(thresh, 
					 contours, 
					 hierarchy, 
					 CV_RETR_EXTERNAL, 
					 CV_CHAIN_APPROX_NONE);

	// find contour with largest area - this doesn't mean we've found the tool,
	//    just narrowing it down to avoid shade and shadow
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

		bool tool_found = false;
//		if (biggest_contour != -1) {
		if (biggest_contour > -1 && max_area > 20000) {
//			approxPolyDP(contours[biggest_contour], 
//						 approx,
//						 arcLength(contours[biggest_contour], true)*.02, 
//						 true);
			// call out to our find_<shape> function 
			tool_found = call(biggest_contour);
			if (tool_found == true) {
				drawContours(viewport, contours, biggest_contour, color, 1);
				putText(frame, 
						str_shape, 
						Point(20, 460), 
						FONT_HERSHEY_PLAIN, 
						2.0, 
						color, 
						2);
				// put secondary tool aquisition code here;
				do {
					capture >> frame;
					putText(frame,
							"looking for top",
							Point(20, 30),
							FONT_HERSHEY_PLAIN,
							2.0,
							CV_RGB(0x00, 0x99, 0x99),
							2);
					rectangle(frame, ROI_tool, CV_RGB(0x00, 0x99, 0x99), 1);
					viewport = frame(ROI_tool);
					viewport.copyTo(thresh);
					cvtColor(thresh, thresh, CV_RGB2GRAY);
					threshold(thresh, thresh, 80, 255, CV_THRESH_BINARY);
					bitwise_xor(thresh, Scalar(255), thresh);
					tool_found = false;
					contours.erase(contours.begin(), contours.end());
					switch(tool) {
						case SQUARE: case TRIANGLE: {
							findContours(thresh, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE, Point(0, 0));
							max_area = 0.0f;
							biggest_contour = -1;
							for (size_t jth = 0; jth < contours.size(); jth++) {
								double tmp_area = contourArea(contours[jth]);
								if (tmp_area > max_area) {
									max_area = tmp_area;
									biggest_contour = jth;
								}
							}
							if (biggest_contour > -1) {
								vector<Point> approx;
								approxPolyDP(contours[biggest_contour],
											 approx,
											 // after experimentation, anthing between .01 and .002 is adequate
											 arcLength(contours[biggest_contour], true)*.01,
											 true);
								switch(approx.size()) {
									case 3: {
										if (tool == TRIANGLE) {
											drawContours(viewport,
														 contours, 
														 biggest_contour,
														 // light yellow
														 CV_RGB(0xFF, 0xFB, 0x73),
														 1);
										// put geometry code for finding center position here
										//    then draw a circle representing the hole.
										//    from this - can we find distance?
										// should be able to find distance, since at this point
										//    we know how large the triangle shape is.
										//
										// how to find the center of a triangle.
										Moments mu;
										mu = moments(contours[biggest_contour], false);
										Point2f mc;
										mc = Point2f( mu.m10 / mu.m00 , mu.m01 / mu.m00 );
										circle(viewport, mc, 5, CV_RGB(0xC5, 0x61, 0xD3), CV_FILLED);
										}
										} // end case 3
										break;
									case 4: {
										if (tool == SQUARE) {
											drawContours(viewport,
														 contours, 
														 biggest_contour,
														 // light purple
														 CV_RGB(0xC5, 0x61, 0xD3),
														 1);
										// as above - put center geometry here.
										// then determine distance.
										Moments mu;
										mu = moments(contours[biggest_contour], false);
										Point2f mc;
										mc = Point2f( mu.m10 / mu.m00 , mu.m01 / mu.m00 );
										circle(viewport, mc, 5, CV_RGB(0xFF, 0xFB, 0x73), CV_FILLED);
										
										}
										}
										break;
									default:
										tool_found = false;
										break;
								}
							}
							}
							break;
						case CIRCLE: {
							// no idea if this works -- NEEDS testing.
							vector<Vec3f> circles;
							HoughCircles(thresh,
										 circles,
										 CV_HOUGH_GRADIENT,
										 2,
										 thresh.rows / 4,
										 200,
										 100);
							double max_radius = 0.0f;
							int biggest_circle = -1;
							for (size_t cth = 0; cth < circles.size(); cth++) {
								if (circles[cth][2] > max_radius) {
									max_radius = circles[cth][2];
									biggest_circle = cth;
								}
							}
							if (biggest_circle > -1) {
								// here's where the center finding code would go.
							//	tool_found = true;
								Point center(cvRound(circles[biggest_circle][0]), cvRound(circles[biggest_circle][1]));
								circle(viewport, center, 5, CV_RGB(0xFF, 0x76, 0x40), CV_FILLED);
							}
							}
							break;
						default:
							tool_found = false;
							break;
					}
					imshow("frame", frame);
					if (waitKey(30) == 27) {
						break;
					}
				} while(tool_found == false); //*/
			}
		}
		imshow("frame", frame);
		if(waitKey(30) == 27) {
			break;
		}
	}
	return(0);
}
