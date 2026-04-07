#ifndef SFX_T_CALIBRATOR_H
#define SFX_T_CALIBRATOR_H

#include "absl/status/statusor.h"
#include "calibration/intrinsic_calibration.pb.h"
#include "opencv2/opencv.hpp"

namespace sfx {

// Looks for a chessboard in the camera and tries to calibrate.
absl::StatusOr<proto::IntrinsicCalibration> CalibrateFromVideo(
    int32_t device_id);

}  // namespace sfx

#endif  // SFX_T_CALIBRATOR_H
