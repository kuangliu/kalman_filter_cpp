#pragma once

#include <iostream>
#include <memory>

#include "kalman_filter.h"

class PointTracker {
 public:
  void Init();

  cv::Mat Process(const cv::Mat& u, const cv::Mat& y) { return kf_.Step(u, y); }

  void SetTime(const double dT);

 private:
  LinearKalmanFilter kf_;
};
