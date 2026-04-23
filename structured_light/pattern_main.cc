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
ABSL_FLAG(int32_t, camera_id, 2, "Camera ID, usually 0 or 2");

absl::Status Run() {
  const auto kSize = cv::Size(640, 480);

  // Make sure that camera and writer are ok.
  cv::VideoCapture cap(absl::GetFlag(FLAGS_camera_id));
  if (!cap.isOpened()) {
    return absl::NotFoundError(absl::StrFormat("Failed to open camera: '%i'.",
                                               absl::GetFlag(FLAGS_camera_id)));
  }

  // Disable all camera automatic features
  cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);
  cap.set(cv::CAP_PROP_EXPOSURE, -5);
  cap.set(cv::CAP_PROP_AUTO_WB, 0);
  cap.set(cv::CAP_PROP_WB_TEMPERATURE, 4500);
  cap.set(cv::CAP_PROP_AUTOFOCUS, 0);

  cv::VideoWriter writer;
  if (!absl::GetFlag(FLAGS_output_video_path).empty()) {
    const int32_t fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
    const int32_t fps = 10;
    if (!writer.open(absl::GetFlag(FLAGS_output_video_path), fourcc, fps, kSize,
                     /*isColor=*/false)) {
      return absl::InternalError("Failed to open output video");
    }
  }

  cv::structured_light::GrayCodePattern::Params params;
  params.width = kSize.width;
  params.height = kSize.height;
  const auto graycode = cv::structured_light::GrayCodePattern::create(params);
  std::vector<cv::Mat> patterns;
  graycode->setWhiteThreshold(50); // Ignore pixels with low contrast
  graycode->generate(patterns);

  // All-white and all-black images needed for shadow mask computation.
  cv::Mat white;
  cv::Mat black;
  graycode->getImagesForShadowMasks(black, white);
  patterns.push_back(white);
  patterns.push_back(black);

  constexpr std::string_view kWindow = "Pattern";
  cv::namedWindow(kWindow.data(), cv::WINDOW_FREERATIO);

  // Move this window to the projector
  cv::imshow(kWindow.data(), patterns[0]);
  cv::waitKey(0);
  cv::pollKey();

  // Clear buffer
  cv::Mat dummy;
  for(int i = 0; i < 5; ++i) cap.read(dummy);

  std::vector<cv::Mat> camera_patterns;
  for (const auto& pattern : patterns) {
    if (writer.isOpened()) writer.write(pattern);
    cv::imshow("Pattern", pattern);
    cv::waitKey(20); // Wait for the projector
    cv::Mat camera_frame;
    if (cap.read(camera_frame)) {
      cv::Mat camera_gray;
      cv::cvtColor(camera_frame, camera_gray, cv::COLOR_BGR2GRAY);
      camera_patterns.emplace_back(camera_gray.clone());
    }
  }

  auto log_projector_pixels = [&](int32_t x, int32_t y) {
    cv::Point projector_pixel;
    if (!graycode->getProjPixel(camera_patterns, x, y, projector_pixel)) {
      LOG(INFO) << "Not detected";
    }
    LOG(INFO) << absl::StreamFormat("Image: (%i, %i) => Projector: (%i, %i)", x,
                                    y, projector_pixel.x, projector_pixel.y);
  };

  // Image: (0, 0) => Projector: (926, 349)
  // Image: (639, 479) => Projector: (402, 434)
  // Image: (320, 240) => Projector: (74, 166)
  log_projector_pixels(0, 0);
  log_projector_pixels(kSize.width - 1, kSize.height - 1);
  log_projector_pixels(kSize.width / 2, kSize.height / 2);

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