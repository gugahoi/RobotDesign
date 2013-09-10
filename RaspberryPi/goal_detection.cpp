#include <cv.h>
#include <highgui.h>

using namespace cv;
using namespace std;

int lowerH=43;
int lowerS=53;
int lowerV=72;

int upperH=82;
int upperS=185;
int upperV=126;

IplImage* GetThresholdedImage(IplImage* imgHSV){
 IplImage* imgThresh=cvCreateImage(cvGetSize(imgHSV),IPL_DEPTH_8U, 1);
 cvInRangeS(imgHSV, cvScalar(lowerH,lowerS,lowerV), cvScalar(upperH,upperS,upperV), imgThresh);
 return imgThresh;
}

void trackObject(IplImage* imgThresh, IplImage* realImage){
    // Calculate the moments of 'imgThresh'
    CvMoments *moments = (CvMoments*)malloc(sizeof(CvMoments));
    cvMoments(imgThresh, moments, 1);
    double moment10 = cvGetSpatialMoment(moments, 1, 0);
    double moment01 = cvGetSpatialMoment(moments, 0, 1);
    double area = cvGetCentralMoment(moments, 0, 0);

    if(area>100){
        int posX = moment10/area;
        int posY = moment01/area;

        if(posX >= 0 && posY >= 0)
        {
            // Draw a pink line from the previous point to the current point
            cvLine(realImage, cvPoint(posX-10, posY), cvPoint(posX+10, posY), cvScalar(255,0,255), 4);
            cvLine(realImage, cvPoint(posX, posY+10), cvPoint(posX, posY-10), cvScalar(255,0,255), 4);

            /* TODO
             * ====
             * To improve certainty that the goal was found, check for the black line and the red LED vertically straight from
             * the center point.
             */
        }
    }
    free(moments);
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
        trackObject(imgThresh, destination);

        cvShowImage("Ball", imgThresh);
        cvShowImage("Video", destination);

        waitKey(80);
    }
    return 0;
}
