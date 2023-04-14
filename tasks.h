
/*
 * Di Zhang
 * Mar 20, 2023
 * CS5330 - Computer Vision
 */

// header functions

// detect and extract and draw chessboard corners
int chessboard_corners(cv::Mat &src, char save, std::vector<std::vector<cv::Point2f>> &corners_list, 
                       cv::Mat &mostrecent_img, std::vector<cv::Point2f> &mostrecent_corner_set, 
                       std::vector<cv::Vec3f> &mostrecent_point_set, std::vector<std::vector<cv::Vec3f>> &point_list);

// calibrate the camera
int calibration(cv::Mat &src, std::vector<std::vector<cv::Point2f>> &corners_list, std::vector<std::vector<cv::Vec3f>> &point_list, 
                cv::Mat &cameraMatrix, cv::Mat &distCoeffs);

// calculate current position of camera
int cal_camera_position(cv::Mat &src, std::vector<cv::Vec3f> &point_set, std::vector<cv::Point2f> &corner_set,
                        bool found);

// create virtual object
int sift(cv::Mat &src);

// detect aruco markers
int aruco(cv::Mat &src);