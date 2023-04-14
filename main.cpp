
/*
 * Di Zhang
 * Mar 20, 2023
 * CS5330 - Computer Vision
 */

#include <opencv2/opencv.hpp>

#include "tasks.h"

int main(int argc, char* argv[]) {
    
    // capture live cam video stream
    cv::VideoCapture *capdev;

    // open the video device
    capdev = new cv::VideoCapture(0);

    // error checking
    if (!capdev -> isOpened()) {
        printf("Unable to open video device");
        return(-1);
    }

    // get some properties of the image
    cv::Size refs((int)capdev -> get(cv::CAP_PROP_FRAME_WIDTH),
                  (int)capdev -> get(cv::CAP_PROP_FRAME_HEIGHT));
    printf("Expected size %d %d\n", refs.width, refs.height);

    // video window
    cv::namedWindow("Video", 1);

    // video frame
    cv::Mat frame;

    // set corner list variable
    std::vector<std::vector<cv::Point2f>> corners_list;

    // set 3d point set and list variables
	std::vector<std::vector<cv::Vec3f>> point_list;


    // most recent corners and image
    cv::Mat mostrecent_img;
    std::vector<cv::Point2f> mostrecent_corner_set;
    std::vector<cv::Vec3f> mostrecent_point_set;

    // initilize camera matrix and distribution coeffients
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;

    // checker
    char checker;

    // no. of command line parameters
    int noarg = argc;
    for (;;) {
        if (noarg > 1) {
            frame = cv::imread(argv[1]);
        } else {
            *capdev >> frame;
        }

        // press q/Q to quit the video stream
        char key = cv::waitKey(10);

        // calculate chessboard corners
        bool found = chessboard_corners(frame, key, corners_list, mostrecent_img, mostrecent_corner_set, mostrecent_point_set, point_list);

        // detect aruco markers
        aruco(frame);

        if (key == 'c' || key == 'C') {
            // calibrate camera
            cv::Mat calib_frame;
            calib_frame = frame.clone();
            calibration(calib_frame, corners_list, point_list, cameraMatrix, distCoeffs);
        }

        if (key == 'p' || key == 'P' || checker == 'p' || checker == 'P') {
            cal_camera_position(frame, mostrecent_point_set, mostrecent_corner_set, found);
            checker = 'p';
        }

        if (key == 'v' || key == 'V' || checker == 'v' || checker == 'V') {
            sift(frame);
            checker = 'v';
        }

        if (key == 'z' || key == 'Z') {
            checker = '\0';
        }

        // show video stream
        cv::imshow("Video", frame);

        if (key == 'q' || key == 'Q') {
            break;
        }
    }
    return 0;
}