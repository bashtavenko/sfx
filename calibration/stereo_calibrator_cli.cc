#include <fstream>
#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "calibrator.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "proto_utils.h"
#include "status_macros.h"

ABSL_FLAG(int32_t, left_camera_id, 0, "Left camera USB id.");
ABSL_FLAG(int32_t, right_camera_id, 2, "Right camera USB id.");
ABSL_FLAG(std::string, output_calibration_proto,
          "/tmp/stereo_calibration.txtpb",
          "Output path for the stereo calibration");

absl::Status Run() {
  ASSIGN_OR_RETURN(
      auto proto_calibration,
      sfx::StereoCalibrationFromVideo(absl::GetFlag(FLAGS_left_camera_id),
                                      absl::GetFlag(FLAGS_right_camera_id)));
  RETURN_IF_ERROR(sfx::WriteProtoToTextProto(
      proto_calibration, absl::GetFlag(FLAGS_output_calibration_proto)));
  LOG(INFO) << absl::StreamFormat(
      "Calibration from camera %i and camera %i was completed and saved to %s.",
      absl::GetFlag(FLAGS_left_camera_id), absl::GetFlag(FLAGS_right_camera_id),
      absl::GetFlag(FLAGS_output_calibration_proto));

  return absl::OkStatus();
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  absl::ParseCommandLine(argc, argv);
  gflags::SetCommandLineOption("logtostderr", "1");
  if (const auto status = Run(); !status.ok()) {
    LOG(ERROR) << status.message();
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}