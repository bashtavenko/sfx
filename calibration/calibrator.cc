#include "calibration/calibrator.h"
#include "calibration/intrinsic_calibration.h"
#include "calibration/intrinsic_calibration.pb.h"
#include "glog/logging.h"
#include "opencv2/imgproc.hpp"
#include "proto_utils.h"

namespace sfx {

static constexpr int32_t kBoardWidth = 6;
static constexpr int32_t kBoardHeight = 9;
const cv::Size board(kBoardWidth, kBoardHeight);

static std::vector<std::vector<cv::Point3f>> GenerateObjectPoints() {
  std::vector<std::vector<cv::Point3f>> result;
  const cv::Size board(kBoardWidth, kBoardHeight);
  std::vector<cv::Point3f> obj;
  for (int i = 0; i < kBoardHeight; ++i) {
    for (int j = 0; j < kBoardWidth; ++j) {
      obj.emplace_back(j, i, 0.0f);
    }
  }
  result.push_back(obj);
  return result;
}

absl::StatusOr<IntrinsicCalibration> CalibrateFromMat(const cv::Mat& image) {
  std::vector<cv::Point2f> corners;
  if (!cv::findChessboardCorners(image, board, corners)) {
    return absl::InternalError("Chessboard not found.");
  }

  std::vector<std::vector<cv::Point2f>> image_points;

  cv::cornerSubPix(
      image, corners, cv::Size(11, 11), cv::Size(-1, -1),
      cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30,
                       0.1));
  image_points.push_back(corners);

  IntrinsicCalibration intrinsic_calibration;
  cv::Mat camera_matrix;
  cv::Mat distortions;

  const double reprojection_error = cv::calibrateCamera(
      GenerateObjectPoints(), image_points, image.size(),
      intrinsic_calibration.camera_matrix,
      intrinsic_calibration.distortion_params, cv::noArray(), cv::noArray());
  intrinsic_calibration.reprojection_error = reprojection_error;
  return intrinsic_calibration;
}

absl::StatusOr<proto::IntrinsicCalibration> CalibrateFromVideo(
    int32_t camera_id) {
  cv::VideoCapture capture(camera_id);
  if (!capture.isOpened()) {
    return absl::NotFoundError(
        absl::StrFormat("Failed to open camera: '%i'.", camera_id));
  }

  constexpr int32_t kNumberOfFrames = 10;
  int32_t number_of_frames = 0;
  cv::Mat frame;
  absl::StatusOr<IntrinsicCalibration> intrinsic_calibration;

  while (capture.read(frame) && number_of_frames < kNumberOfFrames &&
         !intrinsic_calibration.ok()) {
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    intrinsic_calibration = CalibrateFromMat(gray);
    ++number_of_frames;
  }
  if (!intrinsic_calibration.ok()) {
    return absl::InternalError(absl::StrFormat(
        "Could not calibrate after %i frames - %s.", kNumberOfFrames,
        intrinsic_calibration.status().message()));
  }

  return (ConvertIntrinsicCalibrationToProto(intrinsic_calibration.value()));
}

absl::StatusOr<StereoCalibration> StereoCalibrateFromMat(
    const cv::Mat& left_image, const cv::Mat& right_image) {
  std::vector<cv::Point2f> corners;
  if (!cv::findChessboardCorners(left_image, board, corners)) {
    return absl::InternalError("Chessboard for the left camera not found.");
  }
  std::vector<std::vector<cv::Point2f>> left_image_points;
  left_image_points.emplace_back(corners);
  if (!cv::findChessboardCorners(right_image, board, corners)) {
    return absl::InternalError("Chessboard for the right camera not found.");
  }
  std::vector<std::vector<cv::Point2f>> right_image_points;
  right_image_points.emplace_back(corners);

  // The default is cv::CALIB_FIX_INTRINSIC, which is NOT we want
  // It tells to compute intrinsic from scratch.
  constexpr int32_t flags = 0;

  StereoCalibration stereo_calibration;
  stereo_calibration.reprojection_error = cv::stereoCalibrate(
      GenerateObjectPoints(), left_image_points, right_image_points,
      stereo_calibration.left_camera_matrix,
      stereo_calibration.left_distortion_params,
      stereo_calibration.right_camera_matrix,
      stereo_calibration.right_distortion_params, left_image.size(),
      stereo_calibration.R, stereo_calibration.T, stereo_calibration.E,
      stereo_calibration.F, flags);

  if (stereo_calibration.left_camera_matrix.at<double>(0, 0) == 1.) {
    return absl::InternalError("No intrinsic calibration found.");
  }

  //
  // LOG(INFO) << "DBG:" << stereo_calibration.left_camera_matrix.at<double>(0,
  // 0) << " "
  // << stereo_calibration.left_camera_matrix.at<double>(0, 1) << " " <<
  // stereo_calibration.left_camera_matrix.i

  return stereo_calibration;
}

absl::StatusOr<proto::StereoCalibration> StereoCalibrationFromVideo(
    int32_t left_camera_id, int32_t right_camera_id) {
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

  constexpr int32_t kNumberOfFrames = 10;
  int32_t number_of_frames = 0;
  cv::Mat left_frame;
  cv::Mat right_frame;
  bool calibrated = false;

  absl::StatusOr<StereoCalibration> stereo_calibration;
  while (left_cam.read(left_frame) && right_cam.read(right_frame) &&
         number_of_frames < kNumberOfFrames && !calibrated) {
    cv::Mat left_frame_gray;
    cv::cvtColor(left_frame, left_frame_gray, cv::COLOR_BGR2GRAY);
    cv::Mat right_frame_gray;
    cv::cvtColor(right_frame, right_frame_gray, cv::COLOR_BGR2GRAY);
    stereo_calibration =
        StereoCalibrateFromMat(left_frame_gray, right_frame_gray);
    ++number_of_frames;
  }
  if (!stereo_calibration.ok()) {
    return absl::InternalError(absl::StrFormat(
        "Could not calibrate after %i frames - %s.", kNumberOfFrames,
        stereo_calibration.status().message()));
  }

  return ConvertStereoCalibrationToProto(stereo_calibration.value());
}

}  // namespace sfx
