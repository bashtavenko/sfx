#include "calibration/calibrator.h"
#include "calibration/intrinsic_calibration.h"
#include "calibration/intrinsic_calibration.pb.h"
#include "opencv2/imgproc.hpp"
#include "proto_utils.h"

namespace sfx {

static constexpr int32_t kBoardWidth = 6;
static constexpr int32_t kBoardHeight = 9;
const cv::Size board(kBoardWidth, kBoardHeight);

absl::StatusOr<IntrinsicCalibration> CalibrateFromMat(const cv::Mat& image) {
  std::vector<cv::Point2f> corners;
  if (!cv::findChessboardCorners(image, board, corners)) {
    return absl::InternalError("Chessboard not found.");
  }

  std::vector<std::vector<cv::Point3f>> object_points;
  std::vector<std::vector<cv::Point2f>> image_points;
  const cv::Size board(kBoardWidth, kBoardHeight);
  std::vector<cv::Point3f> obj;
  for (int i = 0; i < kBoardHeight; ++i) {
    for (int j = 0; j < kBoardWidth; ++j) {
      obj.emplace_back(j, i, 0.0f);
    }
  }

  cv::cornerSubPix(
      image, corners, cv::Size(11, 11), cv::Size(-1, -1),
      cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30,
                       0.1));

  image_points.push_back(corners);
  object_points.push_back(obj);

  IntrinsicCalibration intrinsic_calibration;
  cv::Mat camera_matrix;
  cv::Mat distortions;
  const double reprojection_error = cv::calibrateCamera(
      object_points, image_points, image.size(),
      intrinsic_calibration.camera_matrix,
      intrinsic_calibration.distortion_params, cv::noArray(), cv::noArray());
  intrinsic_calibration.reprojection_error = reprojection_error;
  return intrinsic_calibration;
}

absl::StatusOr<proto::IntrinsicCalibration> CalibrateFromVideo(
    int32_t device_id) {
  cv::VideoCapture capture(device_id);
  if (!capture.isOpened()) {
    return absl::NotFoundError(
        absl::StrFormat("Failed to open camera: '%i'.", device_id));
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
        "Could not find a chessboard after %i frames.", kNumberOfFrames));
  }

  return (ConvertIntrinsicCalibrationToProto(intrinsic_calibration.value()));
}

}  // namespace sfx