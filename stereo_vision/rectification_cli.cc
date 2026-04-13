#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "calibration/proto_utils.h"
#include "calibration/stereo_calibration.pb.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "status_macros.h"
#include "stereo_vision/rectification.h"

ABSL_FLAG(int32_t, left_camera_id, 0, "Left camera USB id.");
ABSL_FLAG(int32_t, right_camera_id, 2, "Right camera USB id.");
ABSL_FLAG(std::string, stereo_calibration_proto,
          "/tmp/stereo_calibration.txtpb",
          "Stereo calibration input file in the proto text format.");
ABSL_FLAG(std::string, output_rectification_proto, "/tmp/rectification.txtpb",
          "Output path for the rectification file in the proto text format.");

absl::Status Run() {
  ASSIGN_OR_RETURN(auto proto_stereo_calibration,
                   sfx::LoadFromTextProtoFile<sfx::proto::StereoCalibration>(
                       absl::GetFlag(FLAGS_stereo_calibration_proto)));
  sfx::StereoCalibration sfo_stereo_calibration =
      sfx::ConvertStereoCalibrationFromProto(proto_stereo_calibration);
  RETURN_IF_ERROR(sfx::Rectify(absl::GetFlag(FLAGS_left_camera_id),
    absl::GetFlag(FLAGS_right_camera_id), sfo_stereo_calibration));

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