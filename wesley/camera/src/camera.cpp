#include <highgui.h>
#include <cv.h>

using namespace cv;

int main() {
	VideoCapture cap(0);
	if (!cap.isOpened()) {
		std::cerr << "unable to open camera\n";
		return(1);
	}

	Mat frame;
	namedWindow("image raw", CV_WINDOW_AUTOSIZE);

	for (;;) {
		cap >> frame;
		imshow("image raw", frame);

		if (waitKey(30) == 27) {
			break;
		}
	}

	cap.release();
	return(0);
}
