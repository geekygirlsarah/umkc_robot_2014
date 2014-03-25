#include <highgui.h>
#include <cv.h>

using namespace cv;

int main() {
	VideoCapture cap(0);
	if (!cap.isOpened()) {
		std::cerr << "unable to open camera\n";
		return(1);
	}

	cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);

	Mat frame;
	namedWindow("image raw", CV_WINDOW_AUTOSIZE);

	for (;;) {
		cap >> frame;
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
