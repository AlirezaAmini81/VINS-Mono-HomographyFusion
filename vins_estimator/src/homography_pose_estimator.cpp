#include "homography_pose_estimator.h"

HomographyPoseEstimator::HomographyPoseEstimator(const cv::Mat& K) {
    // Ensure K is double precision and 3x3
    if (K.empty() || K.rows != 3 || K.cols != 3) {
        throw std::runtime_error("HomographyPoseEstimator: K must be 3x3 non-empty matrix");
    }
    // Make a copy in CV_64F
    if (K.type() != CV_64F) {
        K.convertTo(K_, CV_64F);
    } else {
        K_ = K.clone();
    }
}

bool HomographyPoseEstimator::estimatePose(const std::vector<cv::Point2f>& pts1,
                                           const std::vector<cv::Point2f>& pts2,
                                           cv::Mat& R_out,
                                           cv::Mat& t_out,
                                           double ransac_thresh)
{
    // Basic checks
    if (pts1.size() < 4 || pts2.size() < 4 || pts1.size() != pts2.size()) {
        return false;
    }

    // findHomography expects vectors of corresponding pixel points (u,v)
    cv::Mat inlier_mask;
    cv::Mat H = cv::findHomography(pts1, pts2, cv::RANSAC, ransac_thresh, inlier_mask);

    if (H.empty()) {
        return false;
    }

    // Decompose homography into possible (R, t, normal) solutions
    std::vector<cv::Mat> rotations, translations, normals;
    int nsol = cv::decomposeHomographyMat(H, K_, rotations, translations, normals);
    if (nsol <= 0) {
        return false;
    }

    // NOTE: selects the first candidate from decomposeHomographyMat without
    // a cheirality check. See "Known Limitations" in README — this is the
    // main planned improvement, not yet implemented.
    rotations[0].convertTo(R_out, CV_64F);
    translations[0].convertTo(t_out, CV_64F);

    // Normalize translation to get direction only (homography has no scale)
    double norm_t = cv::norm(t_out);
    if (norm_t > 1e-9) {
        t_out = t_out / norm_t;
    } else {
        // degenerate translation -> fail
        return false;
    }

    return true;
}
