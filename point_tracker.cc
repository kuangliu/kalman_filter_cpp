#include <random>

#include "point_tracker.h"

void PointTracker::Init() {
  // Transition State Matrix A
  // Note: set dT at each processing step!
  // [ 1 0 dT 0  ]
  // [ 0 1 0  dT ]
  // [ 0 0 1  0  ]
  // [ 0 0 0  1  ]
  cv::Mat A = cv::Mat::eye(4, 4, CV_32F);
  cv::Mat B = cv::Mat::zeros(4, 4, CV_32F);

  // Measure Matrix C
  // [ 1 0 0 0 ]
  // [ 0 1 0 0 ]
  cv::Mat C = (cv::Mat_<float>(2, 4) << 1, 0, 0, 0, 0, 1, 0, 0);

  cv::Mat P = cv::Mat::eye(4, 4, CV_32F);
  cv::Mat Q = cv::Mat::eye(4, 4, CV_32F) * 1e-5;
  cv::Mat R = cv::Mat::eye(2, 2, CV_32F) * 1e-1;

  cv::Mat x = (cv::Mat_<float>(4, 1) << 0, 0, 0, 0);

  kf_ = LinearKalmanFilter(A, B, C, P, Q, R, x);
}

void PointTracker::SetTime(const double dT) {
  kf_.A_.at<float>(0, 2) = dT;
  kf_.A_.at<float>(1, 3) = dT;
}

int main() {
  PointTracker tracker;

  double x0 = 10;
  double y0 = 10;
  double vx = 5;
  double vy = 2;

  std::default_random_engine generator;
  std::normal_distribution<double> gaussian_noise(0, 10.0);

  tracker.Init();

  double d1 = 0;
  double d2 = 0;
  for (int t = 0; t < 100; ++t) {
    double xt = x0 + vx * t;
    double yt = y0 + vy * t;

    double noisy_xt = xt + gaussian_noise(generator);
    double noisy_yt = yt + gaussian_noise(generator);

    cv::Mat y(2, 1, CV_32F);
    y.at<float>(0) = noisy_xt;
    y.at<float>(1) = noisy_yt;

    tracker.SetTime(1);

    cv::Mat u = (cv::Mat_<float>(4, 1) << 0, 0, 0, 0);
    cv::Mat state = tracker.Process(u, y);

    std::cout << "Step: " << t << std::endl;
    std::cout << int(xt) << "," << int(yt) << "  " << int(noisy_xt) << ","
              << int(noisy_yt) << "  " << int(state.at<float>(0)) << ","
              << int(state.at<float>(1)) << std::endl;

    d1 += abs(noisy_xt - xt) + abs(noisy_yt - yt);
    d2 += abs(state.at<float>(0) - xt) + abs(state.at<float>(1) - yt);
  }

  std::cout << d1 << std::endl;
  std::cout << d2 << std::endl;

  return 0;
}
