#ifndef VINS_HOMOGRAPHY_POSE_ESTIMATOR_H
#define VINS_HOMOGRAPHY_POSE_ESTIMATOR_H

#include <opencv2/opencv.hpp>
#include <vector>

// Small helper class to estimate relative pose from homography.
// - Construct with camera intrinsics K (3x3, CV_64F).
// - estimatePose takes matched pixel points in two images (pts1, pts2)
//   and returns R (3x3 CV_64F) and t (3x1 CV_64F, normalized direction).
class HomographyPoseEstimator {
public:
    // Constructor: copy K (expected CV_64F 3x3)
    explicit HomographyPoseEstimator(const cv::Mat& K);

    // Estimate pose. Returns true on success.
    // pts1 and pts2 must be same size and in pixel coordinates (u, v).
    bool estimatePose(const std::vector<cv::Point2f>& pts1,
                      const std::vector<cv::Point2f>& pts2,
                      cv::Mat& R_out,
                      cv::Mat& t_out,
                      double ransac_thresh = 3.0);

private:
    cv::Mat K_; // camera intrinsics
};

#endif // VINS_HOMOGRAPHY_POSE_ESTIMATOR_H

