#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace cv;
using namespace std;

#define RGBMode true
#define STRICTMode true

int lowerH, lowerS, lowerV, upperH, upperS, upperV;

void setColors(int mode)
{
    if (mode == 0) // goal
    {
        if(STRICTMode)
        {
            lowerH=97;
            lowerS=167;
            lowerV=233;

            upperH=144;
            upperS=255;
            upperV=255;
        } else if(RGBMode)
        {
            // RGB Values
            lowerH=0;
            lowerS=167;
            lowerV=233;

            upperH=255;
            upperS=255;
            upperV=255;
        } else {
            // HSV Values
            lowerH=0;
            lowerS=0;
            lowerV=178;

            upperH=180;
            upperS=256;
            upperV=256;
        }
    } else if(mode == 1) // red ball
    {
        if(STRICTMode)
        {
            lowerH=1;
            lowerS=0;
            lowerV=90;
            upperH=13;
            upperS=0;
            upperV=120;
        } else {
            lowerH=0;
            lowerS=0;
            lowerV=90;
            upperH=23;
            upperS=0;
            upperV=180;
        }
    } else if(mode == 2) // blue ball
    {
        if(STRICTMode)
        {
            lowerH=50;
            lowerS=0;
            lowerV=9;
            upperH=85;
            upperS=27;
            upperV=28;
        } else {
            lowerH=26;
            lowerS=0;
            lowerV=0;
            upperH=189;
            upperS=61;
            upperV=57;
        }
    }
}

Mat threshold(Mat img){
 Mat imgThresh(320, 240, DataType<int>::type);
 inRange(img, Scalar(lowerH,lowerS,lowerV), Scalar(upperH,upperS,upperV), imgThresh);
 return imgThresh;
}

void trackGoalMoment(Mat imgThresh, Mat realImage){
    Moments m = moments(imgThresh, true);
    //cout << m.m00 << " - " << m.m10/m.m00 << " " << m.m01/m.m00 << endl;

    if(m.m00>800){
        int posX = (int) m.m10/(int) m.m00;
        int posY = (int) m.m01/(int) m.m00;

        if(posX >= 0 && posY >= 0)
        {
            rectangle( realImage, cvPoint(posX-50, posY-20), cvPoint(posX+50, posY+20), cvScalar(255,0,255), -1);
        }

    }
}

void trackBallMoment(Mat imgThresh, Mat realImage){
    Moments m = moments(imgThresh, true);
    //cout << m.m00 << " - " << m.m10/m.m00 << " " << m.m01/m.m00 << endl;

    if(m.m00>200){
        int posX = (int) m.m10/(int) m.m00;
        int posY = (int) m.m01/(int) m.m00;

        if(posX >= 0 && posY >= 0)
        {
            circle( realImage, cvPoint(posX, posY), m.m00/200, cvScalar(255,0,255), -1);
        }
    }
}

void trackGoal(Mat img, Mat thresh)
{
    if(RGBMode)
    {
        thresh = threshold(img);
    } else {
        Mat imgHSV(320, 240, DataType<int>::type);
        cvtColor(img, imgHSV, CV_RGB2HSV);
        thresh = threshold(imgHSV);
    }
    trackGoalMoment(thresh, img);
    imshow("Video", img);
    imshow("Ball", thresh);
}

void trackRedBall(Mat img, Mat thresh)
{
    thresh = threshold(img);
    trackBallMoment(thresh, img);
    imshow("Video", img);
    imshow("Ball", thresh);
}

void trackBlueBall(Mat img, Mat thresh)
{
    thresh = threshold(img);
    trackBallMoment(thresh, img);
    imshow("Video", img);
    imshow("Ball", thresh);
}

void setwindowSettings()
{
  namedWindow("Video");
  namedWindow("Ball");

  createTrackbar("LowerH", "Ball", &lowerH, 255, NULL);
  createTrackbar("UpperH", "Ball", &upperH, 255, NULL);

  createTrackbar("LowerS", "Ball", &lowerS, 255, NULL);
  createTrackbar("UpperS", "Ball", &upperS, 255, NULL);

  createTrackbar("LowerV", "Ball", &lowerV, 255, NULL);
  createTrackbar("UpperV", "Ball", &upperV, 255, NULL);
}

int main(int argc, char** argv)
{
    setColors(0);
    setwindowSettings();

    string filename = "../Test Videos/IMG_1228.mp4";
    Mat frame, half, thresh;
    int mode = 0;
    bool pause = false;
    for(;;)
    {
        VideoCapture capture(filename);
        if( !capture.isOpened() )
            throw "Error when reading video";
        for(;;)
        {
            if(!pause)
            {
                capture >> frame;
                if(frame.empty()) break;
            }
            resize(frame, half, Size(320,240));

            // ---- Start Image Proc ---- //
            if(mode == 0)
            {
                trackGoal(half, thresh);
            } else if(mode == 1)
            {
                trackRedBall(half, thresh);
            } else if(mode == 2)
            {
                trackBlueBall(half, thresh);
            }
            // ---- Finish Image Proc --- //

            int c = waitKey(50); // waits to display frame
            switch(c)
            {
                case 27: return 0; // exited loop (code 0)
                case 49: mode = 0; setColors(mode); break; // goal capture
                case 50: mode = 1; setColors(mode); break; // red ball tracking
                case 51: mode = 2; setColors(mode); break; // blue ball tracking
                case 32: pause = !pause;
            }
        }
    }
    return 1; // finished code (code 1)
}
