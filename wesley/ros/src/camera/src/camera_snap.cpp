/** CAMERA_SNAP.CPP
 *  by: Eric M Gonzalez
 *  
 *  PURPOSE: This code provides a simple output frame to view what the camera
 *           sees.
 * 
 *           To run this code, call:
 * 
 *              camera_snapshot             -- display raw image with not markers or information
 *              camera_snapshot -r			-- this shows where the camera will look for the rigs
 *              camera_snapshot -t			-- this shows the frame used when identifying tools
 *              camera_snapshot -t s        -- as -t above and also displays contour information
 * 
 *           pressing [Print Screen] will take copy the current frame to "./snapshot.png"
 *              successive snapshots will overwrite previous ones.
 *           pressing [ESC] will exit the program,
 */

#include <highgui.h>
#include <cv.h>

using namespace cv;

int main(int argc, char* argv[]) {
	// attempt to open the camera
	VideoCapture cap(0);
	if (!cap.isOpened()) {
		std::cerr << "unable to open camera\n";
		return(1);
	}

	// look to see user wants us to draw contours
	bool search_con = false;
	if (argc >= 3) {
		if (argv[2][0] == 's') {
			search_con = true;
		}
	}

	// only ever needed this in this program.
	//    this explictly sets the WidthXHeight of the camera's frame.
	cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);

	Mat frame;
	// create out output window
	namedWindow("image raw", CV_WINDOW_AUTOSIZE);

	// for ever..
	for (;;) {
		cap >> frame;
		// check to see how we were called. draw approriate rectangles as needed
		//    the following rectangles come directly from id_flame.cpp and id_tool.cpp
		if (argc >= 2) {
			switch(argv[1][0]) {
				// looking at if_flame frames.
				case 'f': case 'r':
					rectangle(frame, Rect(20, 120, 180, 320), CV_RGB(0xCE, 0x83, 0xCE));
					rectangle(frame, Rect(300, 120, 180, 320), CV_RGB(0xCE, 0x83, 0xCE));
					rectangle(frame, Rect(480, 120, 160, 320), CV_RGB(0xCE, 0x83, 0xCE));
					break;
				// looking at id_tool frames.
				case 't': case 'p':
					rectangle(frame, Rect(160, 80, 300, 360), CV_RGB(0xCe, 0x83, 0xCE));
					break;
			}
		}

		// if we were asked to display contour information
		if (search_con) {
			// create the contours vector
			vector<vector<Point> > contours;
			Mat viewport = frame(Rect(160, 80, 300, 360));
			Mat thresh;
			// copy a region of interest from the original frame
			viewport.copyTo(thresh);
			// convert to grayscale, run our threshold on it, and flip the image
			cvtColor(thresh, thresh, CV_RGB2GRAY);
			threshold(thresh, thresh, 80, 255, CV_THRESH_BINARY);
			bitwise_xor(thresh, Scalar(255), thresh);

			findContours(thresh, contours, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);

			// find the contour with the largest area.
			double max_area = 0.0f;
			int biggest_area = -1;
			for (int cth = 0; cth < contours.size(); cth++) {
				double tmp_area = contourArea(contours[cth]);
				if (tmp_area > max_area) {
					max_area = tmp_area;
					biggest_area = cth;
				}
			}
			
			// if a contour was found, draw it in the viewport
			if (biggest_area > -1) {
				// approximate the edges of the contour.
				vector<Point> approx;
				approxPolyDP(contours[biggest_area], approx, arcLength(contours[biggest_area], true) * .015, true);
				// find the area of that approximation
				double area = contourArea(approx, false);
				// draw the approximated polygon into the viewport
				polylines(viewport, approx, true, CV_RGB(125, 189, 26), 1.5);
				
				// write some other information into the frame
				std::stringstream ss;
				ss << "sides: " << approx.size();
				putText(frame, ss.str(), Point (160, 470), FONT_HERSHEY_PLAIN, 1.5, CV_RGB(0xFF, 0xDF, 0x00), 1.5);
				ss.str("");
				ss << "area: " << area;
				putText(frame, ss.str(), Point (280, 470), FONT_HERSHEY_PLAIN, 1.5, CV_RGB(0xFF, 0xDF, 0x00), 1.5);
			}
		}

		imshow("image raw", frame);
		
		int keypress = waitKey(30);
		if (keypress == 27) {		// escape
			break;
		} else if (keypress == 65377) {		// print_screen
			std::cout << "snapshot" << std::endl;
			vector<int> params;
			params.push_back(CV_IMWRITE_PNG_COMPRESSION);
			params.push_back(9);
			imwrite("snapshot.png", frame, params);
		}
	}

	cap.release();
	return(0);
}
