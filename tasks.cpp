
/*
 * Di Zhang
 * Mar 20, 2023
 * CS5330 - Computer Vision
 */

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <iostream>
#include <fstream>

#include "yaml-cpp/yaml.h"

// detect aruco markers
int aruco(cv::Mat &src) {
    cv::Mat overlay_img;
    cv::Mat warpedImage;
    overlay_img = cv::imread("p.jpg");

    // detect overlay image corners
    std::vector<cv::Point2f> imageCorners;
    imageCorners.push_back(cv::Point2f(0, 0));
    imageCorners.push_back(cv::Point2f(overlay_img.cols, 0));
    imageCorners.push_back(cv::Point2f(overlay_img.cols, overlay_img.rows));
    imageCorners.push_back(cv::Point2f(0, overlay_img.rows));

    //cv::Mat imageWithAlpha;
    //cv::cvtColor(overlay_img, imageWithAlpha, cv::COLOR_BGR2BGRA);
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::aruco::ArucoDetector detector(dictionary, detectorParams);

    detectorParams.markerBorderBits = 2;
    detectorParams.adaptiveThreshWinSizeMax = 75;
    detectorParams.adaptiveThreshWinSizeMin = 45;
    detectorParams.adaptiveThreshConstant = 7;
    detectorParams.cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;

    detector.detectMarkers(src, markerCorners, markerIds);

    if (markerIds.size() > 0) {
        cv::Mat warpMatrix = cv::getPerspectiveTransform(imageCorners, markerCorners[0]);
        cv::warpPerspective(overlay_img, warpedImage, warpMatrix, src.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT);
        src = src + warpedImage;
    }
    
    return 0;
}


// calibrate the camera
int calibration(cv::Mat &src, std::vector<std::vector<cv::Point2f>> &corners_list, std::vector<std::vector<cv::Vec3f>> &point_list,
                cv::Mat &cameraMatrix, cv::Mat &distCoeffs) {

    // initilize cameraMatrix
    cameraMatrix = cv::Mat::eye(3, 3, CV_64FC1);
    cameraMatrix.at<double>(0, 2) = double(src.cols/2);
    cameraMatrix.at<double>(1, 2) = double(src.rows/2);
    distCoeffs = cv::Mat::zeros(1, 5, CV_64FC1);

    // print out the cameraMatrix and distCoeffs to compare after calibrateCamera
    std::cout << "cameraMatrix before: " << cameraMatrix << std::endl;
    std::cout << "distCoeffs before: " << distCoeffs << std::endl;    

    cv::Mat rvecs, tvecs;
    cv::Mat std_deviations_intrinsic, std_deviations_extrinsic, per_view_errors;
    int CALIB_FIX_ASPECT_RATIO;
    double reprojection_error = cv::calibrateCamera(point_list, corners_list, cv::Size(src.cols, src.rows), cameraMatrix, distCoeffs, rvecs, tvecs,
                                                    std_deviations_intrinsic, std_deviations_extrinsic, per_view_errors, CALIB_FIX_ASPECT_RATIO, 
                                                    cv::TermCriteria(cv::TermCriteria::EPS+cv::TermCriteria::MAX_ITER, 30, 0.1));

    std::cout << "cameraMatrix after: " << cameraMatrix << std::endl;
    std::cout << "distCoeffs after: " << distCoeffs << std::endl;
    std::cout << "reprojection_error: " << reprojection_error << std::endl;        

    cv::FileStorage fs_camera_mat("cameraMatrix.yaml", cv::FileStorage::WRITE);
    fs_camera_mat << "cameraMatrix" << cameraMatrix;
    fs_camera_mat.release();

    // save distCoeffs
    cv::FileStorage fs_dist("distCoeffs.yaml", cv::FileStorage::WRITE);
    fs_dist << "distCoeffs" << distCoeffs;
    fs_dist.release();

    return 0;
}

// calculate current position of camera
int cal_camera_position(cv::Mat &src, std::vector<cv::Vec3f> &point_set, std::vector<cv::Point2f> &corner_set,
                        bool found) {
    
    cv::Size board_size(6, 9);
    // initilize cameraMatrix, distCoeffs, rvecs, tvecs
    cv::Mat cameraMatrix, distCoeffs;
    cv::Mat rvecs, tvecs;

    // get cameraMatrix, distCoeffs from file
    cv::FileStorage cameraMatrix_fs("cameraMatrix.yaml", cv::FileStorage::READ);
    cameraMatrix_fs["cameraMatrix"] >> cameraMatrix;
    cv::FileStorage distCoeffs_fs("distCoeffs.yaml", cv::FileStorage::READ);
    distCoeffs_fs["distCoeffs"] >> distCoeffs;

    // check if corners are found in the image
    if (found) {

        cv::Mat mask(src.size(), CV_8UC1, cv::Scalar(255));
        cv::drawChessboardCorners(mask, board_size, corner_set, found);
        cv::inpaint(src, mask, src, 5, cv::INPAINT_TELEA);

        // calculate rotation vector and translation vector
        cv::solvePnP(point_set, corner_set, cameraMatrix, distCoeffs, rvecs, tvecs);

        // calculate rotation vector and translation vector
        std::cout << "rotation vector: " << rvecs << std::endl;
        std::cout << "translation vector: " << tvecs << std::endl;

        // Define the endpoints of the 3D axis
        std::vector<cv::Point3f> axisPoints;
        axisPoints.emplace_back(0.0f, 0.0f, 0.0f);
        axisPoints.emplace_back(2.0f, 0.0f, 0.0f);
        axisPoints.emplace_back(0.0f, 2.0f, 0.0f);
        axisPoints.emplace_back(0.0f, 0.0f, 2.0f);

        // Define the colors of the axes
        std::vector<cv::Scalar> axesColors;
        axesColors.push_back(cv::Scalar(0, 0, 255)); // red for x-axis
        axesColors.push_back(cv::Scalar(0, 255, 0)); // green for y-axis
        axesColors.push_back(cv::Scalar(255, 0, 0)); // blue for z-axis

        // project 3d points to 2d plane
        std::vector<cv::Point2f> projectedPoints;
        cv::projectPoints(axisPoints, rvecs, tvecs, cameraMatrix, distCoeffs, projectedPoints);

        // Draw the projected points on the image
        cv::line(src, projectedPoints[0], projectedPoints[3], axesColors[2], 10);
        cv::line(src, projectedPoints[0], projectedPoints[2], axesColors[1], 10);
        cv::line(src, projectedPoints[0], projectedPoints[1], axesColors[0], 10); 

        // Define the endpoints of the 3D axis for the 'Y' 3D shape
        std::vector<cv::Point3f> axisPoints_y;
        axisPoints_y.emplace_back(2.0f, 5.0f, 0.0f); // 0
        axisPoints_y.emplace_back(3.0f, 5.0f, 0.0f); // 1
        axisPoints_y.emplace_back(6.0f, 5.0f, 0.0f); // 2
        axisPoints_y.emplace_back(7.0f, 5.0f, 0.0f); // 3
        axisPoints_y.emplace_back(4.0f, 4.0f, 0.0f); // 4
        axisPoints_y.emplace_back(5.0f, 4.0f, 0.0f); // 5
        axisPoints_y.emplace_back(4.0f, 3.0f, 0.0f); // 6
        axisPoints_y.emplace_back(5.0f, 3.0f, 0.0f); // 7
        axisPoints_y.emplace_back(4.0f, 0.0f, 0.0f); // 8
        axisPoints_y.emplace_back(5.0f, 0.0f, 0.0f); // 9

        axisPoints_y.emplace_back(2.0f, 5.0f, 1.0f); // 10
        axisPoints_y.emplace_back(3.0f, 5.0f, 1.0f); // 11
        axisPoints_y.emplace_back(6.0f, 5.0f, 1.0f); // 12
        axisPoints_y.emplace_back(7.0f, 5.0f, 1.0f); // 13
        axisPoints_y.emplace_back(4.0f, 4.0f, 1.0f); // 14
        axisPoints_y.emplace_back(5.0f, 4.0f, 1.0f); // 15
        axisPoints_y.emplace_back(4.0f, 3.0f, 1.0f); // 16
        axisPoints_y.emplace_back(5.0f, 3.0f, 1.0f); // 17
        axisPoints_y.emplace_back(4.0f, 0.0f, 1.0f); // 18
        axisPoints_y.emplace_back(5.0f, 0.0f, 1.0f); // 19

        std::vector<cv::Point2f> projectedPoints_y;
        cv::projectPoints(axisPoints_y, rvecs, tvecs, cameraMatrix, distCoeffs, projectedPoints_y);
        // Draw the projected points on the image 
        cv::line(src, projectedPoints_y[0], projectedPoints_y[1], axesColors[1], 5); 
        cv::line(src, projectedPoints_y[1], projectedPoints_y[4], axesColors[1], 5); 
        cv::line(src, projectedPoints_y[4], projectedPoints_y[5], axesColors[1], 5); 
        cv::line(src, projectedPoints_y[5], projectedPoints_y[2], axesColors[1], 5); 
        cv::line(src, projectedPoints_y[2], projectedPoints_y[3], axesColors[1], 5); 
        cv::line(src, projectedPoints_y[3], projectedPoints_y[7], axesColors[1], 5); 
        cv::line(src, projectedPoints_y[7], projectedPoints_y[9], axesColors[1], 5); 
        cv::line(src, projectedPoints_y[9], projectedPoints_y[8], axesColors[1], 5); 
        cv::line(src, projectedPoints_y[8], projectedPoints_y[6], axesColors[1], 5); 
        cv::line(src, projectedPoints_y[6], projectedPoints_y[0], axesColors[1], 5); 

        cv::line(src, projectedPoints_y[10], projectedPoints_y[11], axesColors[1], 5); 
        cv::line(src, projectedPoints_y[11], projectedPoints_y[14], axesColors[1], 5); 
        cv::line(src, projectedPoints_y[14], projectedPoints_y[15], axesColors[1], 5); 
        cv::line(src, projectedPoints_y[15], projectedPoints_y[12], axesColors[1], 5); 
        cv::line(src, projectedPoints_y[12], projectedPoints_y[13], axesColors[1], 5); 
        cv::line(src, projectedPoints_y[13], projectedPoints_y[17], axesColors[1], 5); 
        cv::line(src, projectedPoints_y[17], projectedPoints_y[19], axesColors[1], 5); 
        cv::line(src, projectedPoints_y[19], projectedPoints_y[18], axesColors[1], 5); 
        cv::line(src, projectedPoints_y[18], projectedPoints_y[16], axesColors[1], 5); 
        cv::line(src, projectedPoints_y[16], projectedPoints_y[10], axesColors[1], 5);
        
        cv::line(src, projectedPoints_y[0], projectedPoints_y[10], axesColors[1], 5); 
        cv::line(src, projectedPoints_y[1], projectedPoints_y[11], axesColors[1], 5); 
        cv::line(src, projectedPoints_y[2], projectedPoints_y[12], axesColors[1], 5); 
        cv::line(src, projectedPoints_y[3], projectedPoints_y[13], axesColors[1], 5); 
        cv::line(src, projectedPoints_y[4], projectedPoints_y[14], axesColors[1], 5); 
        cv::line(src, projectedPoints_y[5], projectedPoints_y[15], axesColors[1], 5); 
        cv::line(src, projectedPoints_y[6], projectedPoints_y[16], axesColors[1], 5); 
        cv::line(src, projectedPoints_y[7], projectedPoints_y[17], axesColors[1], 5); 
        cv::line(src, projectedPoints_y[8], projectedPoints_y[18], axesColors[1], 5); 
        cv::line(src, projectedPoints_y[9], projectedPoints_y[19], axesColors[1], 5);

        // Define the vertices of the polygon to be filled
        std::vector<cv::Point> pts_top;
        pts_top.push_back(projectedPoints_y[10]);
        pts_top.push_back(projectedPoints_y[16]);
        pts_top.push_back(projectedPoints_y[18]);
        pts_top.push_back(projectedPoints_y[19]);
        pts_top.push_back(projectedPoints_y[17]);
        pts_top.push_back(projectedPoints_y[13]);
        pts_top.push_back(projectedPoints_y[12]);
        pts_top.push_back(projectedPoints_y[15]);
        pts_top.push_back(projectedPoints_y[14]);
        pts_top.push_back(projectedPoints_y[11]);

        std::vector<cv::Point> pts_left;
        pts_left.push_back(projectedPoints_y[0]);
        pts_left.push_back(projectedPoints_y[6]);
        pts_left.push_back(projectedPoints_y[8]);
        pts_left.push_back(projectedPoints_y[18]);
        pts_left.push_back(projectedPoints_y[16]);
        pts_left.push_back(projectedPoints_y[10]);

        std::vector<cv::Point> pts_bottom;
        pts_bottom.push_back(projectedPoints_y[8]);
        pts_bottom.push_back(projectedPoints_y[9]);
        pts_bottom.push_back(projectedPoints_y[19]);
        pts_bottom.push_back(projectedPoints_y[18]);

        std::vector<cv::Point> pts_right;
        pts_right.push_back(projectedPoints_y[3]);
        pts_right.push_back(projectedPoints_y[7]);
        pts_right.push_back(projectedPoints_y[9]);
        pts_right.push_back(projectedPoints_y[19]);
        pts_right.push_back(projectedPoints_y[17]);
        pts_right.push_back(projectedPoints_y[13]);
        
        std::vector<cv::Point> pts_up;
        pts_up.push_back(projectedPoints_y[0]);
        pts_up.push_back(projectedPoints_y[1]);
        pts_up.push_back(projectedPoints_y[4]);
        pts_up.push_back(projectedPoints_y[5]);
        pts_up.push_back(projectedPoints_y[2]);
        pts_up.push_back(projectedPoints_y[3]);
        pts_up.push_back(projectedPoints_y[13]);
        pts_up.push_back(projectedPoints_y[12]);
        pts_up.push_back(projectedPoints_y[15]);
        pts_up.push_back(projectedPoints_y[14]);
        pts_up.push_back(projectedPoints_y[11]);
        pts_up.push_back(projectedPoints_y[10]);
        
        cv::fillPoly(src, std::vector<std::vector<cv::Point>>({ pts_top }), cv::Scalar(0, 0, 0));
        cv::fillPoly(src, std::vector<std::vector<cv::Point>>({ pts_left }), cv::Scalar(0, 0, 0));
        cv::fillPoly(src, std::vector<std::vector<cv::Point>>({ pts_bottom }), cv::Scalar(0, 0, 0));
        cv::fillPoly(src, std::vector<std::vector<cv::Point>>({ pts_right }), cv::Scalar(0, 0, 0));
        cv::fillPoly(src, std::vector<std::vector<cv::Point>>({ pts_up }), cv::Scalar(0, 0, 0));

    }
    return 0;
}


// detect and extract and draw chessboard corners
bool chessboard_corners(cv::Mat &src, char save, std::vector<std::vector<cv::Point2f>> &corners_list, 
                       cv::Mat &mostrecent_img, std::vector<cv::Point2f> &mostrecent_corner_set, 
                       std::vector<cv::Vec3f> &mostrecent_point_set, std::vector<std::vector<cv::Vec3f>> &point_list) {
    // initialize board length and width
    int WIDTH = 6;
    int LENGTH = 9;

    // define the board size
    cv::Size boardsize(WIDTH, LENGTH);
    
    // set corner set variable
    std::vector<cv::Point2f> corner_set(0);
    std::vector<cv::Point2f> corner_set1(0);
    std::vector<cv::Vec3f> point_set(0);
    
    // convert source image to grayscale image
    cv::Mat gray;
    cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
    

    int chessboardFlags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE;
    // check if the corners are found
    bool found = cv::findChessboardCorners(gray, boardsize, corner_set, chessboardFlags);

    // check if chessboard is in the images
    if (found) {
        // find the corners
        cv::cornerSubPix(gray, corner_set, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS+cv::TermCriteria::MAX_ITER, 30, 0.1));
    
        // draw the corners in the images
        cv::drawChessboardCorners(src, boardsize, corner_set, found);
    
        // print the number of corners and first corner coordinates
        std::cout << "Number of corners present in the image: " << corner_set.size() << std::endl;
        std::cout << "Coordinates of the first corner: " << "(" << corner_set[0].x << "," << corner_set[0].y << ")" << std::endl;
    } else {
        std::cout << "No corners detected" << std::endl;
    }

    // construct 3d position point set
    cv::Vec3f point_3d(0.0f, 0.0f, 0.0f);
    if (point_set.empty()) {
        // construct 3d position set
        for (int i = 0; i < LENGTH; i++) {
            point_3d.val[0] = i;
            for (int j = 0; j < WIDTH; j++) {
                point_3d.val[1] = j;
                //point_3d.val[2] = 0.0;
                point_set.push_back(point_3d);
            }
        }
    }

    // save the most recent image and corners to a different variable
    if (!corner_set.empty()) {
        mostrecent_img = src.clone();
        mostrecent_corner_set = corner_set;
        mostrecent_point_set = point_set;
    }

    // save the corner coordinates when user press s/S
    if (save == 's' || save == 'S') {
        corners_list.push_back(mostrecent_corner_set);
        point_list.push_back(mostrecent_point_set);

        // save the image with corners highlighted
        cv::imwrite("highlighted.png", src);
    }

    cv::Mat src_clone = src.clone();
    cal_camera_position(src_clone, point_set, corner_set, found);
    cv::Mat gray_clone;
    cv::cvtColor(src_clone, gray_clone, cv::COLOR_BGR2GRAY);

    bool found_clone = cv::findChessboardCorners(gray_clone, boardsize, corner_set, chessboardFlags);

    // check if chessboard is in the images
    if (found_clone) {
        // find the corners
        cv::cornerSubPix(gray_clone, corner_set, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS+cv::TermCriteria::MAX_ITER, 30, 0.1));
    
        // draw the corners in the images
        cv::drawChessboardCorners(src, boardsize, corner_set, found_clone);
    
        // print the number of corners and first corner coordinates
        std::cout << "Number of corners present in the image: " << corner_set.size() << std::endl;
        std::cout << "Coordinates of the first corner: " << "(" << corner_set[0].x << "," << corner_set[0].y << ")" << std::endl;
    }
    return found;
}


// use sift to detect images features
int sift(cv::Mat &src) {
    cv::Mat grayImg;
    cv::cvtColor(src, grayImg, cv::COLOR_BGR2GRAY);

    // Create SIFT object
    cv::Ptr<cv::SIFT> sift = cv::SIFT::create();

    // Detect keypoints
    std::vector<cv::KeyPoint> keypoints;
    sift->detect(grayImg, keypoints);

    // Draw keypoints on image
    cv::Mat imgWithKeypoints;
    drawKeypoints(src, keypoints, src, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    return 0;
}

