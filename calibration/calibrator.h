#ifndef SFX_T_CALIBRATOR_H
#define SFX_T_CALIBRATOR_H

#include "absl/status/statusor.h"
#include "calibration/intrinsic_calibration.h"
#include "calibration/stereo_calibration.pb.h"

namespace sfx {

// Looks for a chessboard in the camera and tries to calibrate.
absl::StatusOr<proto::IntrinsicCalibration> CalibrateFromVideo(
    int32_t camera_id);

// Looks for a chessboard that can be seen from both cameras and tries to
// calibrate and fixing intrinsic eguesses.
absl::StatusOr<proto::StereoCalibration> StereoCalibrationFromVideo(
    int32_t left_camera_id, int32_t right_camera_id,
    const IntrinsicCalibration& left_intrinsic,
    const IntrinsicCalibration& right_intrinsic);

// Assuming that OpenCV calibration grid is projected through a projector and
// camera_id sees that pattern, this does the following:
// 1. Intrinsic and extrinsic calibration for the actual camera.
// 2. Back-project pattern object points to the projection points.
// 3. Intrinsic calibrate of the projector.
// 4. Stereo calibrate projector+camera rig
// 5. Returns the same calibration information as per stereo vision.
absl::StatusOr<proto::StereoCalibration> StructuredLightCalibration(
    int32_t camera_id);

// Back-project the image points to the object points given camera matrix, R, T.
// The points are in the format of frame-points (2D or 3D).
// This is required for the structured light to calibrate "projector".
// The R and T dimensions are of Kx3x1 where K is the number of frames.
absl::StatusOr<std::vector<std::vector<cv::Point3f>>> BackProject(
    const cv::Mat& camera_matrix,
    const cv::Mat& distortion_params,
    const cv::Mat& R,
    const cv::Mat& T,
    const std::vector<std::vector<cv::Point2f>>& image_points);
}  // namespace sfx

#endif  // SFX_T_CALIBRATOR_H
