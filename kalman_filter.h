#pragma once

#include <opencv2/core/core.hpp>

class LinearKalmanFilter {
 public:
  LinearKalmanFilter() = default;

  LinearKalmanFilter(const cv::Mat& A,
                     const cv::Mat& B,
                     const cv::Mat& C,
                     const cv::Mat& P,
                     const cv::Mat& Q,
                     const cv::Mat& R,
                     const cv::Mat& x)
      : A_(A), B_(B), C_(C), P_(P), Q_(Q), R_(R), x_(x) {}

  cv::Mat Step(const cv::Mat& u, const cv::Mat& y) {
    Predict(u);
    Update(y);
    return x_;
  }

  void Predict(const cv::Mat& u) {
    x_ = A_ * x_ + B_ * u;
    P_ = A_ * P_ * A_.t() + Q_;
  }

  void Update(const cv::Mat& y) {
    cv::Mat tmp = P_ * C_.t();
    K_ = tmp * ((C_ * tmp) + R_).inv();
    x_ = x_ + K_ * (y - C_ * x_);
    P_ = (cv::Mat::eye(x_.rows, x_.rows, y.type()) - K_ * C_) * P_;
  }

 public:
  cv::Mat A_, B_, C_, P_, Q_, R_, K_;
  cv::Mat x_;
};
