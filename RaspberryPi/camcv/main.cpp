/**
 Author: Gustavo Hoirisch
 Date: Aug 2013
 Sources: http://thinkrpi.wordpress.com/2013/06/15/opencvpi-cam-step-7-face-recognition/
 
 Sample call:
 ./camcv <time> <Hmin> <Smin> <Vmin> <Hmax> <Smax> <Vmax> <debug>
 ./camcv         --> calls with default values - time = 10, debug = 0)
 ./camcv 20      --> default color range and debug = 0 but time = 20 seconds)
 ./camcv 20 100 80 80 180 200 200 1
 --> 20 seconds,
 Range in HSV (100, 80, 80)->(180, 200, 200),
 debug = 1 -> shows threshholded/binary image (signifcant drop in fps)
 ============
 =HSV Values=
 ============
 Pink -> (120, 80, 80)->(150, 256, 256)
 Light Green -> (40, 40, 40) -> (60, 256, 256)
 Red Ball -> (0 150 200) -> (255 255 255)
 Blue Ball -> (0 80 0) -> (10 255 255)
 
 ===============================
 =Camera's Technical Parameters=
 ===============================
 Sensor type: OmniVision OV5647[2] Color CMOS QSXGA (5-megapixel)
 Sensor size: 3.67 x 2.74 mm
 Pixel Count: 2592 x 1944
 Pixel Size: 1.4 x 1.4 um
 Lens: f=3.6 mm, f/2.9
 Angle of View: 54 x 41 degrees
 Field of View: 2.0 x 1.33 m at 2 m
 Full-frame SLR lens equivalent: 35 mm
 Fixed Focus: 1 m to infinity
 Video: 1080p at 30 fps with codec H.264 (AVC)
 Board size: 25 x 24 mm (not including flex cable)
 */

#include "arduino_ctrl.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <memory.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <time.h>
#include <semaphore.h>

extern "C" {
#include "bcm_host.h"
#include "interface/vcos/vcos.h"
    
#include "interface/mmal/mmal.h"
#include "interface/mmal/mmal_logging.h"
#include "interface/mmal/mmal_buffer.h"
#include "interface/mmal/util/mmal_util.h"
#include "interface/mmal/util/mmal_util_params.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/mmal/util/mmal_connection.h"
    
#include "RaspiCamControl.h"
#include "RaspiPreview.h"
#include "RaspiCLI.h"
}

// OPENCV
#include <iostream>
#include <fstream>
#include <sstream>

#include "opencv2/core/core.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/objdetect/objdetect.hpp"

using namespace cv;
using namespace std;
///////////////////////
/// Camera number to use - we only have one camera, indexed from 0.
#define CAMERA_NUMBER 0

// Standard port setting for the camera component
#define MMAL_CAMERA_PREVIEW_PORT 0
#define MMAL_CAMERA_VIDEO_PORT 1
#define MMAL_CAMERA_CAPTURE_PORT 2

// Video format information
#define VIDEO_FRAME_RATE_NUM 30
#define VIDEO_FRAME_RATE_DEN 1
#define VIDEO_GRAYSCALE_MODE 0

// Frame
#define FRAME_HEIGHT  240 // use a multiple of 240 (480, 960)
#define FRAME_WIDTH   320 // use a multiple of 320 (640, 1280)

/// Video render needs at least 2 buffers.
#define VIDEO_OUTPUT_BUFFERS_NUM 3

// Max bitrate we allow for recording
const int MAX_BITRATE = 30000000; // 30Mbits/s

// variable to convert I420 frame to IplImage
int nCount = 0;
IplImage *py, *pu, *pv, *pu_big, *pv_big, *image, *dstImage, *imgHSV, *imgTracking, *imgThresh;

// Minimum and maximum HSV range
int Hmin = 0;
int Smin = 150;
int Vmin = 200;
int Hmax = 255;
int Smax = 255;
int Vmax = 255;

int HGoalmin = 180;
int SGoalmin = 255;
int VGoalmin = 255;
int HGoalmax = 255;
int SGoalmax = 255;
int VGoalmax = 255;

bool searchingGoal = false;
bool debug = false;
int duration = 10;

char key;
Mat img, black;

int mode = 0; // 0 find ball, 1 find goal

int mmal_status_to_int(MMAL_STATUS_T status);

/** Structure containing all state information for the current run
 */
typedef struct
{
    int timeout;                        /// Time taken before frame is grabbed and app then shuts down. Units are milliseconds
    int width;                          /// Requested width of image
    int height;                         /// requested height of image
    int bitrate;                        /// Requested bitrate
    int framerate;                      /// Requested frame rate (fps)
    int graymode;            /// capture in gray only (2x faster)
    int immutableInput;      /// Flag to specify whether encoder works in place or creates a new buffer. Result is preview can display either
    /// the camera output or the encoder output (with compression artifacts)
    RASPIPREVIEW_PARAMETERS preview_parameters;   /// Preview setup parameters
    RASPICAM_CAMERA_PARAMETERS camera_parameters; /// Camera setup parameters
    
    MMAL_COMPONENT_T *camera_component;    /// Pointer to the camera component
    MMAL_COMPONENT_T *encoder_component;   /// Pointer to the encoder component
    MMAL_CONNECTION_T *preview_connection; /// Pointer to the connection from camera to preview
    MMAL_CONNECTION_T *encoder_connection; /// Pointer to the connection from camera to encoder
    
    MMAL_POOL_T *video_pool; /// Pointer to the pool of buffers used by encoder output port
    
} RASPIVID_STATE;

/** Struct used to pass information in encoder port userdata to callback
 */
typedef struct
{
    FILE *file_handle;                   /// File handle to write buffer data to.
    VCOS_SEMAPHORE_T complete_semaphore; /// semaphore which is posted when we reach end of frame (indicates end of capture or fault)
    RASPIVID_STATE *pstate;            /// pointer to our state in case required in callback
} PORT_USERDATA;

// default status
static void default_status(RASPIVID_STATE *state)
{
    if (!state)
    {
        vcos_assert(0);
        return;
    }
    
    // Default everything to zero
    memset(state, 0, sizeof(RASPIVID_STATE));
    
    // Now set anything non-zero
    state->timeout           = duration * 1000;
    state->width             = FRAME_WIDTH;
    state->height            = FRAME_HEIGHT;
    state->bitrate           = MAX_BITRATE;
    state->framerate         = VIDEO_FRAME_RATE_NUM;
    state->immutableInput    = 1;
    state->graymode          = VIDEO_GRAYSCALE_MODE;
    
    // Setup preview window defaults
    raspipreview_set_defaults(&state->preview_parameters);
    
    // Set up the camera_parameters to default
    raspicamcontrol_set_defaults(&state->camera_parameters);
}

void cameraAngle(int mode)
{
    for (int x = 0; x<20; x++) {
        if (mode == 0) {
            ctrlServoCamera(BALL);
        } else {
            ctrlServoCamera(FRONT);
        }
    }
}

void closeArms()
{
    for (int x=0; x<10; x++) {
        ctrlServo(RIGHT, CLOSE);
        ctrlServo(LEFT, CLOSE);
    }
}

void openArms()
{
    for (int x=0; x<10; x++) {
        ctrlServo(RIGHT, OPEN);
    }
    for (int x=0; x<10; x++) {
        ctrlServo(LEFT, OPEN);
    }
}

void cameraUp()
{
    for (int x=0; x<10; x++) {
        ctrlServoCamera(FRONT);
    }
}

//This function threshold the HSV image and create a binary image
IplImage *GetThresholdedGoal(IplImage *imgHSV)
{    
    IplImage *imgThresh = cvCreateImage(cvGetSize(imgHSV), IPL_DEPTH_8U, 1);
    cvInRangeS(imgHSV, cvScalar(HGoalmin, SGoalmin, VGoalmin), cvScalar(HGoalmax, SGoalmax, VGoalmax), imgThresh);
    return imgThresh;
}


IplImage *GetThresholdedBall(IplImage *imgHSV)
{
    IplImage *imgThresh = cvCreateImage(cvGetSize(imgHSV), IPL_DEPTH_8U, 1);
    cvInRangeS(imgHSV, cvScalar(Hmin, Smin, Vmin), cvScalar(Hmax, Smax, Vmax), imgThresh);
    return imgThresh;
}

string intToString(int number)
{
    std::stringstream ss;
    ss << number;
    return ss.str();
}

void kick(int x, int y, Mat &frame)
{
    int xDistance = x - (FRAME_WIDTH / 2);
    int yDistance = y - (FRAME_HEIGHT / 2);
    if (xDistance < -60)
    {
        //go left
        ctrlMotor(LEFTSMALL);
    } else if (xDistance > 60)
    {
        //go right
        ctrlMotor(RIGHTSMALL);
    } else {
        //if close to goal kick otherwise go forward
        cout << "Goal is straight forward" << endl;
        openArms();
        ctrlSolenoid(KICK);
        mode = 0;
        searchingGoal = false;
    }   
}

void ballFind(int x, int y)
{
    cout << "Ball in frame" << endl;
    int xDistance = x - (FRAME_WIDTH / 2);
    int yDistance = y - (FRAME_HEIGHT / 2);
    if (xDistance < -60)
    {
        //go left
        ctrlMotor(LEFTSMALL);
    } else if (xDistance > 60)
    {
        //go right
        ctrlMotor(RIGHTSMALL);
    } else {
        //go straight
        if(yDistance < 100)
        {
            cout << "Closing arms" << endl;
            mode = 1;
            closeArms();
            ctrlMotor(BRAKE);
        } else { ctrlMotor(FORWARD); }
    }
}

// Morphological Operations //
void cvOpen(const CvArr *src, CvArr *dst, IplConvKernel *element)
{
    cvErode (src, dst, element, 1);
    cvDilate(src, dst, element, 1);
}

void cvClose(const CvArr *src, CvArr *dst, IplConvKernel *element)
{
    cvDilate(src, dst, element, 1);
    cvErode (src, dst, element, 1);
}
// End Morphological Operations //

void trackBall(IplImage *imgThresh, Mat &cameraFeed)
{
    // Morphological Transforms
    //IplConvKernel *se21 = cvCreateStructuringElementEx(21, 21, 10, 10, CV_SHAPE_RECT, NULL);
    //IplConvKernel *se11 = cvCreateStructuringElementEx(11, 11, 5,  5,  CV_SHAPE_RECT, NULL);
    //cvClose(imgThresh, imgThresh, se21);
    //cvOpen(imgThresh, imgThresh, se11);
    //cvReleaseStructuringElement(&se21);
    //cvReleaseStructuringElement(&se11);
    
    // Calculate the moments of 'imgThresh'
    CvMoments *moments = (CvMoments *)malloc(sizeof(CvMoments));
    cvMoments(imgThresh, moments, 1);
    double moment10 = cvGetSpatialMoment(moments, 1, 0);
    double moment01 = cvGetSpatialMoment(moments, 0, 1);
    double area = cvGetCentralMoment(moments, 0, 0);
    
    if (area > 200)
    {
        // calculate the position of the ball
        int posX = moment10 / area;
        int posY = moment01 / area;
        
        if (posX > -1 && posY > -1)
        {
            ballFind(posX, posY);
        }
    } else {
        ctrlMotor(FORWARD);
        // Change camera angle?
    }
    free(moments);
}

void trackGoal(IplImage *imgThresh, Mat &cameraFeed)
{
    // Calculate the moments of 'imgThresh'
    CvMoments *moments = (CvMoments *)malloc(sizeof(CvMoments));
    cvMoments(imgThresh, moments, 1);
    double moment10 = cvGetSpatialMoment(moments, 1, 0);
    double moment01 = cvGetSpatialMoment(moments, 0, 1);
    double area = cvGetCentralMoment(moments, 0, 0);
    
    if (area > 500)
    {
        // calculate the position of the ball
        int posX = moment10 / area;
        int posY = moment01 / area;
        
        if (posX > -1 && posY > -1)
        {
            //goal found
            kick(posX, posY, cameraFeed);
        }
    }
    free(moments);
}


void initGoalSearch()
{
    cameraAngle(1);
    searchingGoal = true;
}

/**
 *  buffer header callback function for video
 *
 * @param port Pointer to port from which callback originated
 * @param buffer mmal buffer header pointer
 */
static void video_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
    MMAL_BUFFER_HEADER_T *new_buffer;
    PORT_USERDATA *pData = (PORT_USERDATA *)port->userdata;
    
    if (pData)
    {
        
        if (buffer->length)
        {
            
            mmal_buffer_header_mem_lock(buffer);
            
            //
            // *** PR : OPEN CV Stuff here !
            //
            int w = pData->pstate->width; // get image size
            int h = pData->pstate->height;
            int h4 = h / 4;
            
            memcpy(py->imageData, buffer->data, w * h); // read Y
            
            if (pData->pstate->graymode == 0)
            {
                memcpy(pu->imageData, buffer->data + w * h, w * h4); // read U
                memcpy(pv->imageData, buffer->data + w * h + w * h4, w * h4); // read v
                
                cvResize(pu, pu_big, CV_INTER_NN);
                cvResize(pv, pv_big, CV_INTER_NN);  //CV_INTER_LINEAR looks better but it's slower
                cvMerge(py, pu_big, pv_big, NULL, image);
                
                cvCvtColor(image, dstImage, CV_YCrCb2RGB);  // convert in RGB color space (slow)
                cvCvtColor(dstImage, imgHSV, CV_RGB2HSV);  // convert in RGB color space (slow)
                if(mode == 1)
                {
                    // Find goal
                    if(!searchingGoal) initGoalSearch();
                    imgThresh = GetThresholdedGoal(imgHSV);
                    img = cvarrToMat(dstImage);
                    trackGoal(imgThresh, img);
                }
                else
                {
                    // Find ball
                    cameraAngle(0);
                    imgThresh = GetThresholdedBall(imgHSV);
                    img = cvarrToMat(dstImage);
                    trackBall(imgThresh, img);
                }
                
                if (debug)
                {
                    black = cvarrToMat(imgThresh);
                    imshow("Black", black);
                }
            }
            else
            {
                // gray channel, py
                img = cvarrToMat(py);
            }
            
            if(debug) imshow("Video", img);
            key = (char) waitKey(1);
            nCount++;       // count frames displayed
            
            mmal_buffer_header_mem_unlock(buffer);
        }
        else vcos_log_error("buffer null");
        
    }
    else
    {
        vcos_log_error("Received a encoder buffer callback with no state");
    }
    
    // release buffer back to the pool
    mmal_buffer_header_release(buffer);
    
    // and send one back to the port (if still open)
    if (port->is_enabled)
    {
        MMAL_STATUS_T status;
        
        new_buffer = mmal_queue_get(pData->pstate->video_pool->queue);
        
        if (new_buffer)
            status = mmal_port_send_buffer(port, new_buffer);
        
        if (!new_buffer || status != MMAL_SUCCESS)
            vcos_log_error("Unable to return a buffer to the encoder port");
    }
    
}

/**
 * Create the camera component, set up its ports
 *
 * @param state Pointer to state control struct
 *
 * @return 0 if failed, pointer to component if successful
 *
 */
static MMAL_COMPONENT_T *create_camera_component(RASPIVID_STATE *state)
{
    MMAL_COMPONENT_T *camera = 0;
    MMAL_ES_FORMAT_T *format;
    MMAL_PORT_T *preview_port = NULL, *video_port = NULL, *still_port = NULL;
    MMAL_STATUS_T status;
    
    /* Create the component */
    status = mmal_component_create(MMAL_COMPONENT_DEFAULT_CAMERA, &camera);
    
    if (status != MMAL_SUCCESS)
    {
        vcos_log_error("Failed to create camera component");
        goto error;
    }
    
    if (!camera->output_num)
    {
        vcos_log_error("Camera doesn't have output ports");
        goto error;
    }
    
    video_port = camera->output[MMAL_CAMERA_VIDEO_PORT];
    still_port = camera->output[MMAL_CAMERA_CAPTURE_PORT];
    
    //  set up the camera configuration
    {
        MMAL_PARAMETER_CAMERA_CONFIG_T cam_config =
        {
            { MMAL_PARAMETER_CAMERA_CONFIG, sizeof(cam_config) },
            cam_config.max_stills_w = state->width,
            cam_config.max_stills_h = state->height,
            cam_config.stills_yuv422 = 0,
            cam_config.one_shot_stills = 0,
            cam_config.max_preview_video_w = state->width,
            cam_config.max_preview_video_h = state->height,
            cam_config.num_preview_video_frames = 3,
            cam_config.stills_capture_circular_buffer_height = 0,
            cam_config.fast_preview_resume = 0,
            cam_config.use_stc_timestamp = MMAL_PARAM_TIMESTAMP_MODE_RESET_STC
        };
        mmal_port_parameter_set(camera->control, &cam_config.hdr);
    }
    // Set the encode format on the video  port
    
    format = video_port->format;
    format->encoding_variant = MMAL_ENCODING_I420;
    format->encoding = MMAL_ENCODING_I420;
    format->es->video.width = state->width;
    format->es->video.height = state->height;
    format->es->video.crop.x = 0;
    format->es->video.crop.y = 0;
    format->es->video.crop.width = state->width;
    format->es->video.crop.height = state->height;
    format->es->video.frame_rate.num = state->framerate;
    format->es->video.frame_rate.den = VIDEO_FRAME_RATE_DEN;
    
    status = mmal_port_format_commit(video_port);
    if (status)
    {
        vcos_log_error("camera video format couldn't be set");
        goto error;
    }
    
    // PR : plug the callback to the video port
    status = mmal_port_enable(video_port, video_buffer_callback);
    if (status)
    {
        vcos_log_error("camera video callback2 error");
        goto error;
    }
    
    // Ensure there are enough buffers to avoid dropping frames
    if (video_port->buffer_num < VIDEO_OUTPUT_BUFFERS_NUM)
        video_port->buffer_num = VIDEO_OUTPUT_BUFFERS_NUM;
    
    
    // Set the encode format on the still  port
    format = still_port->format;
    format->encoding = MMAL_ENCODING_OPAQUE;
    format->encoding_variant = MMAL_ENCODING_I420;
    format->es->video.width = state->width;
    format->es->video.height = state->height;
    format->es->video.crop.x = 0;
    format->es->video.crop.y = 0;
    format->es->video.crop.width = state->width;
    format->es->video.crop.height = state->height;
    format->es->video.frame_rate.num = 1;
    format->es->video.frame_rate.den = 1;
    
    status = mmal_port_format_commit(still_port);
    if (status)
    {
        vcos_log_error("camera still format couldn't be set");
        goto error;
    }
    
    
    //PR : create pool of message on video port
    MMAL_POOL_T *pool;
    video_port->buffer_size = video_port->buffer_size_recommended;
    video_port->buffer_num = video_port->buffer_num_recommended;
    pool = mmal_port_pool_create(video_port, video_port->buffer_num, video_port->buffer_size);
    if (!pool)
    {
        vcos_log_error("Failed to create buffer header pool for video output port");
    }
    state->video_pool = pool;
    
    /* Ensure there are enough buffers to avoid dropping frames */
    if (still_port->buffer_num < VIDEO_OUTPUT_BUFFERS_NUM)
        still_port->buffer_num = VIDEO_OUTPUT_BUFFERS_NUM;
    
    /* Enable component */
    status = mmal_component_enable(camera);
    
    if (status)
    {
        vcos_log_error("camera component couldn't be enabled");
        goto error;
    }
    
    raspicamcontrol_set_all_parameters(camera, &state->camera_parameters);
    
    state->camera_component = camera;
    
    return camera;
    
error:
    
    if (camera)
        mmal_component_destroy(camera);
    
    return 0;
}

/**
 * Destroy the camera component
 *
 * @param state Pointer to state control struct
 *
 */
static void destroy_camera_component(RASPIVID_STATE *state)
{
    if (state->camera_component)
    {
        mmal_component_destroy(state->camera_component);
        state->camera_component = NULL;
    }
}


/**
 * Destroy the encoder component
 *
 * @param state Pointer to state control struct
 *
 */
static void destroy_encoder_component(RASPIVID_STATE *state)
{
    // Get rid of any port buffers first
    if (state->video_pool)
    {
        mmal_port_pool_destroy(state->encoder_component->output[0], state->video_pool);
    }
    
    if (state->encoder_component)
    {
        mmal_component_destroy(state->encoder_component);
        state->encoder_component = NULL;
    }
}

/**
 * Connect two specific ports together
 *
 * @param output_port Pointer the output port
 * @param input_port Pointer the input port
 * @param Pointer to a mmal connection pointer, reassigned if function successful
 * @return Returns a MMAL_STATUS_T giving result of operation
 *
 */
static MMAL_STATUS_T connect_ports(MMAL_PORT_T *output_port, MMAL_PORT_T *input_port, MMAL_CONNECTION_T **connection)
{
    MMAL_STATUS_T status;
    
    status =  mmal_connection_create(connection, output_port, input_port, MMAL_CONNECTION_FLAG_TUNNELLING | MMAL_CONNECTION_FLAG_ALLOCATION_ON_INPUT);
    
    if (status == MMAL_SUCCESS)
    {
        status =  mmal_connection_enable(*connection);
        if (status != MMAL_SUCCESS)
            mmal_connection_destroy(*connection);
    }
    
    return status;
}

/**
 * Checks if specified port is valid and enabled, then disables it
 *
 * @param port  Pointer the port
 *
 */
static void check_disable_port(MMAL_PORT_T *port)
{
    if (port && port->is_enabled)
        mmal_port_disable(port);
}

/**
 * Handler for sigint signals
 *
 * @param signal_number ID of incoming signal.
 *
 */
static void signal_handler(int signal_number)
{
    vcos_log_error("Aborting program\n");
    exit(255);
}

/**
 * main
 */
int main(int argc, char *argv[])
{
    if (argc > 1)
    {
        cout << "Using input values." << endl;
        duration = atoi(argv[1]);
        if (argc > 2)
        {
            debug = (atoi(argv[2]) == 1 ) ? true : false;
            if (argc > 3)
            {
                Hmin = atoi(argv[3]);
                Smin = atoi(argv[4]);
                Vmin = atoi(argv[5]);
                Hmax = atoi(argv[6]);
                Smax = atoi(argv[7]);
                Vmax = atoi(argv[8]);
            }
        }
    }
    // Our main data storage vessel..
    RASPIVID_STATE state;
    
    MMAL_STATUS_T status;// = -1;
    MMAL_PORT_T *camera_video_port = NULL;
    MMAL_PORT_T *camera_still_port = NULL;
    MMAL_PORT_T *preview_input_port = NULL;
    MMAL_PORT_T *encoder_input_port = NULL;
    MMAL_PORT_T *encoder_output_port = NULL;
    
    time_t timer_begin, timer_end;
    double secondsElapsed;
    
    bcm_host_init();
    signal(SIGINT, signal_handler);
    
    // read default status
    default_status(&state);
    
    // init windows and OpenCV Stuff
    ctrlInit();
    openArms();
    ctrlServoCamera(BALL);
    ctrlServoCamera(BALL);
    ctrlServoCamera(BALL);
    if(debug) cvNamedWindow("Video", CV_WINDOW_AUTOSIZE);
    if (debug) cvNamedWindow("Black", CV_WINDOW_AUTOSIZE);
    dstImage = cvCreateImage(cvSize(FRAME_WIDTH, FRAME_HEIGHT), IPL_DEPTH_8U, 3);
    py = cvCreateImage(cvSize(FRAME_WIDTH, FRAME_HEIGHT), IPL_DEPTH_8U, 1);      // Y component of YUV I420 frame
    pu = cvCreateImage(cvSize(FRAME_WIDTH / 2, FRAME_HEIGHT / 2), IPL_DEPTH_8U, 1); // U component of YUV I420 frame
    pv = cvCreateImage(cvSize(FRAME_WIDTH / 2, FRAME_HEIGHT / 2), IPL_DEPTH_8U, 1); // V component of YUV I420 frame
    pu_big = cvCreateImage(cvSize(FRAME_WIDTH, FRAME_HEIGHT), IPL_DEPTH_8U, 1);
    pv_big = cvCreateImage(cvSize(FRAME_WIDTH, FRAME_HEIGHT), IPL_DEPTH_8U, 1);
    image = cvCreateImage(cvSize(FRAME_WIDTH, FRAME_HEIGHT), IPL_DEPTH_8U, 3);   // final picture to display
    imgHSV = cvCreateImage(cvSize(FRAME_WIDTH, FRAME_HEIGHT), IPL_DEPTH_8U, 3);
    imgTracking = cvCreateImage(cvSize(FRAME_WIDTH, FRAME_HEIGHT), IPL_DEPTH_8U, 3);
    cvZero(imgTracking); //covert the image, 'imgTracking' to black
    
    // create camera
    if (!create_camera_component(&state))
    {
        vcos_log_error("%s: Failed to create camera component", __func__);
    }
    else
    {
        PORT_USERDATA callback_data;
        
        camera_video_port   = state.camera_component->output[MMAL_CAMERA_VIDEO_PORT];
        camera_still_port   = state.camera_component->output[MMAL_CAMERA_CAPTURE_PORT];
        
        VCOS_STATUS_T vcos_status;
        
        callback_data.pstate = &state;
        
        vcos_status = vcos_semaphore_create(&callback_data.complete_semaphore, "RaspiStill-sem", 0);
        vcos_assert(vcos_status == VCOS_SUCCESS);
        
        // assign data to use for callback
        camera_video_port->userdata = (struct MMAL_PORT_USERDATA_T *)&callback_data;
        
        // init timer
        time(&timer_begin);
        
        
        // start capture
        if (mmal_port_parameter_set_boolean(camera_video_port, MMAL_PARAMETER_CAPTURE, 1) != MMAL_SUCCESS)
        {
            return 0;
        }
        
        // Send all the buffers to the video port
        
        int num = mmal_queue_length(state.video_pool->queue);
        int q;
        for (q = 0; q < num; q++)
        {
            MMAL_BUFFER_HEADER_T *buffer = mmal_queue_get(state.video_pool->queue);
            
            if (!buffer)
                vcos_log_error("Unable to get a required buffer %d from pool queue", q);
            
            if (mmal_port_send_buffer(camera_video_port, buffer) != MMAL_SUCCESS)
                vcos_log_error("Unable to send a buffer to encoder output port (%d)", q);
        }
        
        // Now wait until we need to stop
        vcos_sleep(state.timeout);
        
        //mmal_status_to_int(status);
        // Disable all our ports that are not handled by connections
        check_disable_port(camera_still_port);
        
        if (state.camera_component)
            mmal_component_disable(state.camera_component);
        
        //destroy_encoder_component(&state);
        raspipreview_destroy(&state.preview_parameters);
        destroy_camera_component(&state);
    }
    
    time(&timer_end);  /* get current time; same as: timer = time(NULL)  */
    cvReleaseImage(&dstImage);
    cvReleaseImage(&imgHSV);
    cvReleaseImage(&pu);
    cvReleaseImage(&pv);
    cvReleaseImage(&py);
    cvReleaseImage(&pu_big);
    cvReleaseImage(&pv_big);
    cvReleaseImage(&imgTracking);
    cameraAngle(1);
    closeArms();
    
    secondsElapsed = difftime(timer_end, timer_begin);
    
    printf ("%.f seconds for %d frames : FPS = %f\n", secondsElapsed, nCount, (float)((float)(nCount) / secondsElapsed));
    
    return 0;
}

