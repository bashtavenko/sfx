// Structured light stereo calibration

#include <fstream>
#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "calibration/calibrator.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "proto_utils.h"
#include "status_macros.h"

ABSL_FLAG(int32_t, camera_id, 2, "USB camera id.");
ABSL_FLAG(std::string, output_calibration_file, "/tmp/sl_calibration.txtpb",
          "Output path for the structured light stereo calibration");

absl::Status Run() {
  ASSIGN_OR_RETURN(auto proto_calibration, sfx::StructuredLightCalibration(
                                               absl::GetFlag(FLAGS_camera_id)));
  RETURN_IF_ERROR(sfx::WriteProtoToTextProto(
      proto_calibration, absl::GetFlag(FLAGS_output_calibration_file)));

  LOG(INFO) << absl::StreamFormat(
      "Stereo calibration from camera %i and projector was completed and saved "
      "to %s.",
      absl::GetFlag(FLAGS_camera_id),
      absl::GetFlag(FLAGS_output_calibration_file));

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