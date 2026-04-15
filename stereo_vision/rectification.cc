#include "stereo_vision/rectification.h"
#include "glog/logging.h"
#include "opencv2/calib3d.hpp"

namespace sfx {

absl::Status Rectify(const cv::Mat& left_image_input,
                     const cv::Mat& right_image_input,
                     const StereoCalibration& stereo_calibration) {
  // Left and right images are mutable and will be remaped.
  cv::Mat left_image = left_image_input;
  cv::Mat right_image = right_image_input;
  cv::Mat R1;
  cv::Mat R2;
  cv::Mat P1;
  cv::Mat P2;
  const auto image_size = cv::Size(left_image.size());
  cv::stereoRectify(stereo_calibration.left_camera_matrix,
                    stereo_calibration.left_distortion_params,
                    stereo_calibration.right_camera_matrix,
                    stereo_calibration.right_distortion_params, image_size,
                    stereo_calibration.R, stereo_calibration.T, R1, R2, P1, P2,
                    /*Q=*/cv::noArray());

  // Precompute maps for cv::remap()
  cv::Mat map11;
  cv::Mat map12;
  cv::Mat map21;
  cv::Mat map22;
  cv::initUndistortRectifyMap(stereo_calibration.left_camera_matrix,
                              stereo_calibration.left_distortion_params, R1, P1,
                              image_size, CV_16SC2, map11, map12);
  cv::initUndistortRectifyMap(stereo_calibration.right_camera_matrix,
                              stereo_calibration.right_distortion_params, R2,
                              P2, image_size, CV_16SC2, map21, map22);

  cv::Mat left_image_rectified;
  cv::remap(left_image, left_image_rectified, map11, map12, cv::INTER_LINEAR);
  cv::Mat right_image_rectified;
  cv::remap(right_image, right_image_rectified, map21, map22, cv::INTER_LINEAR);

  // Visual check
  cv::Mat stacked;
  cv::hconcat(left_image_rectified, right_image_rectified, stacked);
  for (int y = 0; y < stacked.rows; y += 40) {
    cv::line(stacked, {0, y}, {stacked.cols, y}, cv::Scalar(0, 255, 0), 1);
  }
  cv::imwrite("/tmp/epipolar_check.png", stacked);

  // Get disparity
  cv::Ptr<cv::StereoSGBM> stereo = cv::StereoSGBM::create(
      0,           // minDisparity
      16,          // numDisparities - must be divisible by 16; ~2x expected
      5,           // blockSize - smaller for low-baseline
      8 * 5 * 5,   // P1
      32 * 5 * 5,  // P2
      1, 10, 10, 100, 32, cv::StereoSGBM::MODE_HH);
  cv::Mat disparity;
  stereo->compute(left_image_rectified, right_image_rectified, disparity);
  cv::Mat visual_disparity;
  cv::normalize(disparity, visual_disparity, 0, 256, cv::NORM_MINMAX, CV_8U);

  return absl::OkStatus();
}

absl::Status Rectify(int32_t left_camera_id, int32_t right_camera_id,
                     const StereoCalibration& stereo_calibration) {
  cv::VideoCapture left_cam(left_camera_id);
  if (!left_cam.isOpened()) {
    return absl::NotFoundError(
        absl::StrFormat("Failed to open left camera: %i", left_camera_id));
  }

  cv::VideoCapture right_cam(right_camera_id);
  if (!right_cam.isOpened()) {
    return absl::NotFoundError(
        absl::StrFormat("Failed to open right camera: %i", right_camera_id));
  }

  cv::Mat left_image;
  CHECK(left_cam.read(left_image)) << "Cannot read from left cam.";
  cv::Mat left_image_gray;
  cv::cvtColor(left_image, left_image_gray, cv::COLOR_BGR2GRAY);
  cv::Mat right_image;
  CHECK(right_cam.read(right_image)) << "Cannot read from right cam.";
  cv::Mat right_image_gray;
  cv::cvtColor(right_image, right_image_gray, cv::COLOR_BGR2GRAY);

  return Rectify(left_image_gray, right_image_gray, stereo_calibration);
}

}  // namespace sfx