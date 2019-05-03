#ifndef PATTERNDETECTION_H_INCLUDED
#define PATTERNDETECTION_H_INCLUDED

#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <vector>
#include <fstream>
#include <iomanip>
#include <iostream>

//! Different patterns available
enum class Pattern{CHESSBOARD,CIRCLES_GRID,ASYMMETRIC_CIRCLES_GRID,PYRAMID};

//! Structure for saving calibration parameters
struct CalibParams{

    Pattern calib_pattern=Pattern::CHESSBOARD; //!< type of pattern used for calibrating.
    float element_size=0.002; //!< size of the elements (square size, circle size, etc...).
    float MAX_MRE=0.5; //!< MRE threshold for stopping optimization.
    unsigned int MAX_IMAGES=90; //!< maximum nb of images to be processed. It is used to avoid dealing with too many images and wait for hours.
    unsigned int MIN_IMAGES=20; //!< minimum nb of images to be considered, if lower, optimization stops.
    cv::Size board_sz = cv::Size(8,6); //!< size of the calibartion pattern;
    cv::Size image_size = cv::Size(640,480); //!< size of the images.
    std::string cam_name=""; //!< name of the camera.
    bool display=false; //!< display option. To display of save rectified images.
    int interval=1; //!< interval between to images.
    bool skip_mono_calib = false;
};

//! Sorting function. Sorts keypoints by distance to the centre of the pattern.
bool sortKpt(cv::KeyPoint kpt1, cv::KeyPoint kpt2);
//!Sorting function. Sorts features by distance to the centre of the pattern.
bool sortPt(cv::Point2f pt1, cv::Point2f pt2);
//! Sorting function. Sorts corner features by their angle
bool sortAngle(cv::Point2f kpt1, cv::Point2f kpt2);
//! Sorting function. Sorts the features belonging to the pyramid by their angle.
bool sortAnglePyr(cv::Point2f kpt1, cv::Point2f kpt2);
//! Sorting function. Helps to find the top of the pyramid.
bool findOrigin(const cv::Point2f& pt1, const cv::Point2f& pt2);
//! refine blob centre detection. It is assumed that 2D feature follow a normal distribution. It does not take into account distortions due to projections.
void refineCentreDetection(const cv::Mat& img, std::vector<cv::KeyPoint>& keypoints);
//! Calculates the 3D location of the features forming the calibration pattern
/*! Input: - params, set of calibration parameters.
    Ouput: - corners, the vector of 3D features.*/
void calcPatternPosition(std::vector<cv::Point3f>& corners, CalibParams& params);
//! Displays the pyramid pattern.
/*! Helps to see if the pattern has been well detected or not. */
void drawPyramidPattern(cv::Mat& img, std::vector<cv::Point2f>& centers, bool found);
//! Detect and identify features from the pyramid pattern.
/*! Input: - img, grayscale image.
    Output: - centers, vector of detected features.
            - found, true if detected properly, false otherwise.*/
bool findPyramid(const cv::Mat& img, std::vector<cv::Point2f>& centers);
//! Open images, detect the calibration pattern and display it.
std::vector<std::vector<cv::Point2f>> findPattern(const std::string& pathToImages, std::vector<int>& image_idx, CalibParams& params);
//! Open images, detect the calibration pattern and display it.
void findPatternStereo(const std::string& pathToImages, std::vector<std::vector<cv::Point2f>>& leftPoints, std::vector<std::vector<cv::Point2f>>& rightPoints, CalibParams& params);
//! Computes the Mean Reprojection Error.
/*! reproject 3D points into the images plane and compute the mean error for all the calibration images.*/
double computeMRE(const std::vector<std::vector<cv::Point3f>>& objectPoints, const std::vector<std::vector<cv::Point2f>>& imagePts, const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs, const cv::Mat& K, const cv::Mat& dist, std::vector<double>& repValues);

#endif // PATTERNDETECTION_H_INCLUDED
