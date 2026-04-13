// Rectification and correspondence
#ifndef SFX_RECTIFICATION_H
#define SFX_RECTIFICATION_H
#include "absl/status/status.h"
#include "calibration/stereo_calibration.h"

namespace sfx {

absl::Status Rectify(int32_t left_camera_id, int32_t right_camera_id,
                     const StereoCalibration& stereo_calibration);

}  // namespace sfx

#endif  // SFX_RECTIFICATION_H
