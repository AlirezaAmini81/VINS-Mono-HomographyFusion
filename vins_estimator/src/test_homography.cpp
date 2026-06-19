#include "homography_pose_estimator.h"
#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    // Simple synthetic test: apply a known homography (pure translation in pixels)
    cv::Mat K = (cv::Mat_<double>(3,3) << 300, 0, 320, 0, 300, 240, 0, 0, 1);

    // create a few points in image1
    std::vector<cv::Point2f> pts1 = { {100,100}, {200,100}, {200,200}, {100,200}, {150,150} };
    // move them in image2 by +10 pixels x, +5 pixels y (planar translation)
    std::vector<cv::Point2f> pts2;
    for (auto &p : pts1) pts2.push_back(cv::Point2f(p.x + 10.0f, p.y + 5.0f));

    HomographyPoseEstimator h(K);
    cv::Mat R, t;
    bool ok = h.estimatePose(pts1, pts2, R, t);
    std::cout << "ok=" << ok << "\nR=\n" << R << "\n t=\n" << t << std::endl;
    return 0;
}
