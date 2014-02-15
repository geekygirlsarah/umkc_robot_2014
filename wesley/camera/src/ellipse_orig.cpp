#include <cv.h>
#include <highgui.h>

#include <stdio.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

//
// We need this to be high enough to get rid of things that are too small too
// have a definite shape.  Otherwise, they will end up as ellipse false positives.
//
#define MIN_AREA 100.00    
//
// One way to tell if an object is an ellipse is to look at the relationship
// of its area to its dimensions.  If its actual occupied area can be estimated
// using the well-known area formula Area = PI*A*B, then it has a good chance of
// being an ellipse.
//
// This value is the maximum permissible error between actual and estimated area.
//
#define MAX_TOL  100.00

using namespace cv;

int main( int argc, char** argv )
{
    IplImage *src, *dst;
    Mat frame;
    // open first video camera
    VideoCapture capture(0);
    if (!capture.isOpened()) {
    	std::cerr << "unable to open video camera\n";
    	return(1);
    }

    namedWindow("dest", CV_WINDOW_AUTOSIZE);
    namedWindow("camera", CV_WINDOW_AUTOSIZE);

    capture >> frame; 
    cvtColor(frame, frame, CV_RGB2GRAY);
    src = cvCreateImage(frame.size(), IPL_DEPTH_8U, frame.channels());
    // destination will be grayscale, therefore 1 channel

    CvMemStorage* storage = cvCreateMemStorage(0);
   	CvSeq* contour = 0;    
    // the first command line parameter must be file name of binary (black-n-white) image
    for (;;) {
	    capture >> frame;
    	cvtColor(frame, frame, CV_RGB2GRAY);
	    src->imageData = (char*)frame.data;
    	IplImage* dst  = cvCreateImage( cvGetSize(src), 8, 3 );
	    cvThreshold( src, src, 128, 255, CV_THRESH_BINARY );
    	//
	    // Invert the image such that white is foreground, black is background.
    	// Dilate to get rid of noise.
	    //
    	cvXorS(src, cvScalar(255, 0, 0, 0), src, NULL);
	    cvDilate(src, src, NULL, 2);    
    	cvShowImage("camera", src);
    	cvFindContours( src, storage, &contour, sizeof(CvContour), CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));
//	    cvZero( dst );
		for( ; contour != 0; contour = contour->h_next ) {
	        double actual_area = fabs(cvContourArea(contour, CV_WHOLE_SEQ, 0));
        	if (actual_area < MIN_AREA)
    	        continue;
	        //
        	// FIXME:
    	    // Assuming the axes of the ellipse are vertical/perpendicular.
	        //
        	CvRect rect = ((CvContour *)contour)->rect;
    	    int A = rect.width / 2; 
	        int B = rect.height / 2;
        	double estimated_area = M_PI * A * B;
    	    double error = fabs(actual_area - estimated_area);    
	        if (error > MAX_TOL)
        	    continue;    
	       	//CvScalar color = CV_RGB( rand() % 255, rand() % 255, rand() % 255 );
	       	CvScalar color = CV_RGB(127, 54, 104);
    	    cvDrawContours( dst, contour, color, color, -1, CV_FILLED, 8, cvPoint(0,0));
	    }   //*/
	 	cvShowImage("dest", dst);
	 	if (waitKey(30) == 27) {
   			break;
	   	}
	}

    return(0);
}
