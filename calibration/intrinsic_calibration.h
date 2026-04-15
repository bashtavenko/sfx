#ifndef SFX_T_INTRINSIC_CALIBRATION_H
#define SFX_T_INTRINSIC_CALIBRATION_H
#include "opencv2/opencv.hpp"

namespace sfx {

// Result of calibration
struct IntrinsicCalibration {
  cv::Mat camera_matrix;
  cv::Mat distortion_params;
  double reprojection_error = 0;
};

// Input of calibration
struct CalibrationInput {
  cv::Mat image; // Chessboard
  std::vector<cv::Point2f> corners; // detected corners
};

} // namespace sfx


#endif //SFX_T_INTRINSIC_CALIBRATION_H
