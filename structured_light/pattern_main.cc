// Shows pattern and optionally writes a short movie.
#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "opencv2/opencv.hpp"
#include "opencv2/structured_light/graycodepattern.hpp"
#include "opencv2/structured_light/structured_light.hpp"
#include "status_macros.h"

ABSL_FLAG(std::string, output_video_path, "", "Output movie");

absl::Status Run() {
  const auto kSize = cv::Size(640, 480);

  cv::structured_light::GrayCodePattern::Params params;
  params.width = kSize.width;
  params.height = kSize.height;

  const auto graycode = cv::structured_light::GrayCodePattern::create(params);
  std::vector<cv::Mat> patterns;
  graycode->generate(patterns);

  // All-white and all-black images needed for shadow mask computation.
  cv::Mat white;
  cv::Mat black;
  graycode->getImagesForShadowMasks(black, white);
  patterns.push_back(white);
  patterns.push_back(black);

  cv::VideoWriter writer;
  if (!absl::GetFlag(FLAGS_output_video_path).empty()) {
    const int32_t fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
    const int32_t fps = 10;
    if (!writer.open(absl::GetFlag(FLAGS_output_video_path), fourcc, fps, kSize,
                     /*isColor=*/false)) {
      return absl::InternalError("Failed to open output video");
    }
  }
  for (const auto& pattern : patterns) {
    if (writer.isOpened()) writer.write(pattern);
    cv::imshow("Pattern", pattern);
    cv::waitKey(200); // Wait just to easier to see each pattern.
  }
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