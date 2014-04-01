#include <highgui.h>
#include <cv.h>

using namespace cv;

int main(int argc, char* argv[]) {
	VideoCapture cap(0);
	if (!cap.isOpened()) {
		std::cerr << "unable to open camera\n";
		return(1);
	}

	bool search_con = false;
	if (argc >= 3) {
		if (argv[2][0] == 's') {
			search_con = true;
		}
	}

	cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);

	Mat frame;
	namedWindow("image raw", CV_WINDOW_AUTOSIZE);

	for (;;) {
		cap >> frame;
		if (argc >= 2) {
			switch(argv[1][0]) {
				case 'f': case 'r':
					rectangle(frame, Rect(20, 120, 180, 320), CV_RGB(0xCE, 0x83, 0xCE));
					rectangle(frame, Rect(300, 120, 180, 320), CV_RGB(0xCE, 0x83, 0xCE));
					rectangle(frame, Rect(480, 120, 160, 320), CV_RGB(0xCE, 0x83, 0xCE));
					break;
				case 't': case 'p':
					rectangle(frame, Rect(160, 80, 300, 360), CV_RGB(0xCe, 0x83, 0xCE));
					break;
			}
		}

		if (search_con) {
			vector<vector<Point> > contours;
			Mat viewport = frame(Rect(160, 80, 300, 360));
			Mat thresh;
			viewport.copyTo(thresh);
			cvtColor(thresh, thresh, CV_RGB2GRAY);
			threshold(thresh, thresh, 80, 255, CV_THRESH_BINARY);
			bitwise_xor(thresh, Scalar(255), thresh);

			findContours(thresh, contours, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);

			double max_area = 0.0f;
			int biggest_area = -1;
			for (int cth = 0; cth < contours.size(); cth++) {
				double tmp_area = contourArea(contours[cth]);
				if (tmp_area > max_area) {
					max_area = tmp_area;
					biggest_area = cth;
				}
			}
			
			if (biggest_area > -1) {
				vector<Point> approx;
				approxPolyDP(contours[biggest_area], approx, arcLength(contours[biggest_area], true) * .015, true);

				double area = contourArea(approx, false);
				polylines(viewport, approx, true, CV_RGB(125, 189, 26), 1.5);
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
		if (keypress == 27) {
			break;
		} else if (keypress == 65377) {
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
