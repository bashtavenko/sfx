#ifndef SFX_T_CALIBRATOR_H
#define SFX_T_CALIBRATOR_H

#include "absl/status/statusor.h"
#include "calibration/intrinsic_calibration.pb.h"
#include "calibration/stereo_calibration.pb.h"

namespace sfx {

// Looks for a chessboard in the camera and tries to calibrate.
absl::StatusOr<proto::IntrinsicCalibration> CalibrateFromVideo(
    int32_t camera_id);

// Looks for a chessboard that can be seen from both cameras and tries to
// calibrate. Without intrinsic guesses, calibration may not always work.
absl::StatusOr<proto::StereoCalibration> StereoCalibrationFromVideo(
    int32_t left_camera_id, int32_t right_camera_id);

}  // namespace sfx

#endif  // SFX_T_CALIBRATOR_H
