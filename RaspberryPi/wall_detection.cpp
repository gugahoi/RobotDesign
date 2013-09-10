#include <cv.h>
#include <highgui.h>

using namespace cv;
using namespace std;

int lowerH=7;
int lowerS=190;
int lowerV=75;

int upperH=28;
int upperS=256;
int upperV=210;

IplImage* GetThresholdedImage(IplImage* imgHSV){
    IplImage* imgThresh=cvCreateImage(cvGetSize(imgHSV),IPL_DEPTH_8U, 1);
    cvInRangeS(imgHSV, cvScalar(lowerH,lowerS,lowerV), cvScalar(upperH,upperS,upperV), imgThresh);
    return imgThresh;
}

void setwindowSettings(){
 cvNamedWindow("Video");
 cvNamedWindow("Ball");

 cvCreateTrackbar("LowerH", "Ball", &lowerH, 180, NULL);
 cvCreateTrackbar("UpperH", "Ball", &upperH, 180, NULL);

  cvCreateTrackbar("LowerS", "Ball", &lowerS, 256, NULL);
  cvCreateTrackbar("UpperS", "Ball", &upperS, 256, NULL);

  cvCreateTrackbar("LowerV", "Ball", &lowerV, 256, NULL);
  cvCreateTrackbar("UpperV", "Ball", &upperV, 256, NULL);
}

int main(){

    setwindowSettings();

    IplImage *img, *destination;

    while(true){
        img=cvLoadImage("b.jpg");
        destination = cvCreateImage( cvSize(320, 240), img->depth, img->nChannels );
        cvResize(img, destination);

        IplImage* imgHSV = cvCreateImage(cvGetSize(destination), IPL_DEPTH_8U, 3);
        cvCvtColor(destination, imgHSV, CV_BGR2HSV);

        IplImage* imgThresh = GetThresholdedImage(imgHSV);

        cvShowImage("Ball", imgThresh);
        cvShowImage("Video", destination);

        waitKey(80);
    }
    return 0;
}
