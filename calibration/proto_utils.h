#ifndef SFX_T_PROTO_UTILS_H
#define SFX_T_PROTO_UTILS_H
#include <google/protobuf/message.h>
#include <google/protobuf/text_format.h>
#include <fstream>
#include <string_view>
#include "absl/status//statusor.h"
#include "absl/strings/str_cat.h"
#include "calibration/intrinsic_calibration.h"
#include "calibration/intrinsic_calibration.pb.h"

namespace sfx {

proto::IntrinsicCalibration ConvertIntrinsicCalibrationToProto(
    const IntrinsicCalibration& intrinsic_calibration);

IntrinsicCalibration ConvertIntrinsicCalibrationFromProto(
    const proto::IntrinsicCalibration& proto);

template <typename ProtoType>
absl::StatusOr<ProtoType> LoadFromTextProtoFile(std::string_view file_path) {
  static_assert(std::is_base_of_v<google::protobuf::Message, ProtoType>,
                "ProtoType must be a protobuf message type");

  std::ifstream file(file_path.data());
  if (!file.is_open()) {
    return absl::InvalidArgumentError(
        absl::StrCat("Failed to open file: ", file_path));
  }

  std::string text_proto((std::istreambuf_iterator<char>(file)),
                         std::istreambuf_iterator<char>());

  if (text_proto.empty()) {
    return absl::InvalidArgumentError(absl::StrCat("Empty file: ", file_path));
  }

  ProtoType proto;
  if (!google::protobuf::TextFormat::ParseFromString(text_proto, &proto)) {
    return absl::InternalError(
        absl::StrCat("Failed to parse proto message from file: ", file_path));
  }

  return proto;
}

// Writes proto to the text proto
template <typename ProtoType>
absl::Status WriteProtoToTextProto(ProtoType proto,
                                                  std::string_view file_path) {
  static_assert(std::is_base_of_v<google::protobuf::Message, ProtoType>,
                "ProtoType must be a protobuf message type");
  std::string text_format;
  google::protobuf::TextFormat::PrintToString(proto, &text_format);
  std::ofstream output_file(file_path.data());
  if (!output_file) {
    return absl::InternalError(absl::StrCat("Failed writing to ", file_path));
  }
  output_file << text_format;
  output_file.close();
  return absl::OkStatus();
}

}  // namespace sfx

#endif  // SFX_T_PROTO_UTILS_H
