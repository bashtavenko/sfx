#ifndef SFX_STEREO_CALIBRATION_H
#define SFX_STEREO_CALIBRATION_H
#include "opencv2/opencv.hpp"

namespace sfx {

// Result of cv::stereoCalibrate outputs
struct StereoCalibration {
  cv::Mat left_camera_matrix;
  cv::Mat left_distortion_params;
  cv::Mat right_camera_matrix;
  cv::Mat right_distortion_params;
  // R and T are related to the left camera
  cv::Mat R; // 3x3
  cv::Mat T;  // 3x1
  cv::Mat E;  // 3x3
  cv::Mat F;  // 3x3
  double reprojection_error = 0;
};

} // namespace sfx



#endif //SFX_STEREO_CALIBRATION_H
