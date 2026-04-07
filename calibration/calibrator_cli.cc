#include <fstream>
#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "calibrator.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "proto_utils.h"
#include "status_macros.h"

ABSL_FLAG(int32_t, camera_id, 0, "Camera ID, usually 0 or 1");
ABSL_FLAG(std::string, output_calibration_proto,
          "/tmp/intinsic_calibration.txtpb", "Intrinsic camera calibration");

absl::Status Run() {
  ASSIGN_OR_RETURN(auto proto_calibration,
                   sfx::CalibrateFromVideo(absl::GetFlag(FLAGS_camera_id)));
  RETURN_IF_ERROR(sfx::WriteProtoToTextProto(
      proto_calibration, absl::GetFlag(FLAGS_output_calibration_proto)));
  LOG(INFO) << absl::StreamFormat(
      "Calibration from device %i was completed and saved to %s file.",
      absl::GetFlag(FLAGS_camera_id),
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