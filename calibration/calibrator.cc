#include "calibration/calibrator.h"
#include "calibration/intrinsic_calibration.h"
#include "calibration/intrinsic_calibration.pb.h"
#include "glog/logging.h"
#include "opencv2/imgproc.hpp"
#include "proto_utils.h"
#include "status_macros.h"

namespace sfx {
static constexpr int32_t kBoardWidth = 6;
static constexpr int32_t kBoardHeight = 9;
const cv::Size board(kBoardWidth, kBoardHeight);

static bool ChessboardFound(const cv::Mat& image,
                            std::vector<cv::Point2f>& corners) {
  return cv::findChessboardCorners(image, board, corners);
}

static std::vector<std::vector<cv::Point3f>> GenerateObjectPoints(
    int32_t size = 1, int32_t square_size = 22) {
  std::vector<cv::Point3f> obj;
  for (int32_t i = 0; i < kBoardHeight; ++i) {
    for (int32_t j = 0; j < kBoardWidth; ++j) {
      obj.emplace_back(j * square_size, i * square_size, 0.0f);
    }
  }
  std::vector<std::vector<cv::Point3f>> result(size);
  for (int i = 0; i < size; ++i) {
    result[i] = obj;
  }
  return result;
}

absl::StatusOr<IntrinsicCalibration> CalibrateFromInput(
    const std::vector<CalibrationInput>& input) {
  CHECK(!input.empty());
  const auto image_size = cv::Size(input[0].image.size());

  IntrinsicCalibration intrinsic_calibration;

  std::vector<std::vector<cv::Point2f>> image_points;
  for (const auto& data : input) {
    image_points.emplace_back(data.corners);
  }

  const double reprojection_error = cv::calibrateCamera(
      GenerateObjectPoints(input.size()), image_points, image_size,
      intrinsic_calibration.camera_matrix,
      intrinsic_calibration.distortion_params, cv::noArray(), cv::noArray());
  intrinsic_calibration.reprojection_error = reprojection_error;

  return intrinsic_calibration;
}

// This is only requied for the structured light
absl::StatusOr<IntrinsicCalibration> CalibrateFromInput(
    const std::vector<std::vector<cv::Point3f>>& object_points,
    const std::vector<CalibrationInput>& input, cv::Mat& R, cv::Mat& T) {
  CHECK(!input.empty());
  const auto image_size = cv::Size(input[0].image.size());

  IntrinsicCalibration intrinsic_calibration;

  std::vector<std::vector<cv::Point2f>> image_points;
  for (const auto& data : input) {
    image_points.emplace_back(data.corners);
  }

  const double reprojection_error =
      cv::calibrateCamera(object_points, image_points, image_size,
                          intrinsic_calibration.camera_matrix,
                          intrinsic_calibration.distortion_params, R, T);
  intrinsic_calibration.reprojection_error = reprojection_error;

  return intrinsic_calibration;
}

absl::StatusOr<proto::IntrinsicCalibration> CalibrateFromVideo(
    int32_t camera_id) {
  cv::VideoCapture capture(camera_id);
  if (!capture.isOpened()) {
    return absl::NotFoundError(
        absl::StrFormat("Failed to open camera: '%i'.", camera_id));
  }

  constexpr int32_t kNumberOfFrames = 50;
  auto valid_frames = std::vector<CalibrationInput>(kNumberOfFrames);
  int32_t number_of_frames = 0;
  cv::Mat frame;

  // Get valid image points
  while (capture.read(frame) && number_of_frames < kNumberOfFrames) {
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    std::vector<cv::Point2f> image_points;
    if (ChessboardFound(gray, image_points)) {
      CalibrationInput input{.image = gray, .corners = image_points};
      valid_frames[number_of_frames] = input;
      ++number_of_frames;
    }
  }
  ASSIGN_OR_RETURN(auto intrinsic_calibration,
                   CalibrateFromInput(valid_frames));
  return (ConvertIntrinsicCalibrationToProto(intrinsic_calibration));
}

absl::StatusOr<StereoCalibration> CalibrateFromPairs(
    const std::vector<CalibrationPairs>& pairs,
    const IntrinsicCalibration& left_intrinsic,
    const IntrinsicCalibration& right_intrinsic) {
  CHECK(!pairs.empty());
  const auto image_size = cv::Size(pairs[0].left_frame.size());

  std::vector<std::vector<cv::Point2f>> left_image_points;
  std::vector<std::vector<cv::Point2f>> right_image_points;
  for (const auto& pair : pairs) {
    left_image_points.emplace_back(pair.left_corners);
    right_image_points.emplace_back(pair.right_corners);
  }

  const std::vector<std::vector<cv::Point3f>> object_points =
      GenerateObjectPoints(pairs.size());

  // Copy intrinsic so that cv::StereoCalibrate can fix them.
  constexpr int32_t stereo_flags = cv::CALIB_FIX_INTRINSIC;

  StereoCalibration stereo_calibration;
  stereo_calibration.left_camera_matrix = left_intrinsic.camera_matrix;
  stereo_calibration.left_distortion_params = left_intrinsic.distortion_params;
  stereo_calibration.right_camera_matrix = right_intrinsic.camera_matrix;
  stereo_calibration.right_distortion_params =
      right_intrinsic.distortion_params;
  stereo_calibration.reprojection_error = cv::stereoCalibrate(
      object_points, left_image_points, right_image_points,
      stereo_calibration.left_camera_matrix,
      stereo_calibration.left_distortion_params,
      stereo_calibration.right_camera_matrix,
      stereo_calibration.right_distortion_params, image_size,
      stereo_calibration.R, stereo_calibration.T, stereo_calibration.E,
      stereo_calibration.F, stereo_flags);

  if (stereo_calibration.left_camera_matrix.at<double>(0, 0) == 1.) {
    return absl::InternalError("No intrinsic calibration found.");
  }
  return stereo_calibration;
}

absl::StatusOr<proto::StereoCalibration> StereoCalibrationFromVideo(
    int32_t left_camera_id, int32_t right_camera_id,
    const IntrinsicCalibration& left_intrinsic,
    const IntrinsicCalibration& right_intrinsic) {
  cv::VideoCapture left_cam(left_camera_id);
  if (!left_cam.isOpened()) {
    return absl::NotFoundError(
        absl::StrFormat("Failed to open left camera: %i", left_camera_id));
  }

  cv::VideoCapture right_cam(right_camera_id);
  if (!right_cam.isOpened()) {
    return absl::NotFoundError(
        absl::StrFormat("Failed to open right camera: %i", right_camera_id));
  }

  // Get valid frames in both cameras
  constexpr int32_t kNumberOfFrames = 1;
  auto valid_frames = std::vector<CalibrationPairs>(kNumberOfFrames);

  int32_t number_of_frames = 0;
  cv::Mat left_frame;
  cv::Mat right_frame;

  absl::StatusOr<StereoCalibration> stereo_calibration;

  // Blocks untill if find the required number of valid frames
  while (left_cam.read(left_frame) && right_cam.read(right_frame) &&
         number_of_frames < kNumberOfFrames) {
    cv::Mat left_frame_gray;
    cv::cvtColor(left_frame, left_frame_gray, cv::COLOR_BGR2GRAY);
    cv::Mat right_frame_gray;
    cv::cvtColor(right_frame, right_frame_gray, cv::COLOR_BGR2GRAY);
    CHECK(left_frame_gray.size() == right_frame_gray.size())
        << "Cameras have different resolutions.";
    std::vector<cv::Point2f> left_corners;
    std::vector<cv::Point2f> right_corners;
    if (ChessboardFound(left_frame_gray, left_corners) &&
        ChessboardFound(right_frame_gray, right_corners)) {
      CalibrationPairs pairs{.left_frame = left_frame_gray,
                             .left_corners = left_corners,
                             .right_frame = right_frame_gray,
                             .right_corners = right_corners};
      valid_frames[number_of_frames] = pairs;
      ++number_of_frames;
    }
  }
  stereo_calibration =
      CalibrateFromPairs(valid_frames, left_intrinsic, right_intrinsic);
  if (!stereo_calibration.ok()) {
    return absl::InternalError(absl::StrFormat(
        "Could not calibrate - %s.", stereo_calibration.status().message()));
  }

  return ConvertStereoCalibrationToProto(stereo_calibration.value());
}

absl::StatusOr<proto::StereoCalibration> StructuredLightCalibration(
    int32_t camera_id) {
  cv::VideoCapture capture(camera_id);
  if (!capture.isOpened()) {
    return absl::NotFoundError(
        absl::StrFormat("Failed to open camera: '%i'.", camera_id));
  }

  constexpr int32_t kNumberOfFrames = 10;
  auto image_points = std::vector<std::vector<cv::Point2f>>(kNumberOfFrames);
  int32_t number_of_frames = 0;
  cv::Mat frame;

  // Get valid image points
  while (capture.read(frame) && number_of_frames < kNumberOfFrames) {
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    std::vector<cv::Point2f> corners;
    if (ChessboardFound(gray, corners)) {
      image_points[number_of_frames] = corners;
      ++number_of_frames;
    }
  }
  const cv::Size image_size = frame.size();
  std::vector<std::vector<cv::Point3f>> camera_object_points =
      GenerateObjectPoints(/*size=*/kNumberOfFrames, /*square_size=*/22);
  cv::Mat camera_matrix;
  cv::Mat camera_distortion_params;
  cv::Mat R;
  cv::Mat T;
  cv::calibrateCamera(camera_object_points, image_points, image_size,
                      camera_matrix, camera_distortion_params, R, T);
  // The shape of R and T is nX1 (number of frames) of cv::Point3F
  // return (ConvertIntrinsicCalibrationToProto(intrinsic_calibration));
  ASSIGN_OR_RETURN(auto projector_object_points,
                   sfx::BackProject(camera_matrix, camera_distortion_params, R,
                                    T, image_points));

  cv::Mat projector_camera_matrix;
  cv::Mat projector_distorion_params;
  cv::calibrateCamera(projector_object_points, image_points, image_size,
                      projector_camera_matrix, projector_distorion_params,
                      cv::noArray(), cv::noArray());

  StereoCalibration stereo_calibration;
  stereo_calibration.left_camera_matrix = camera_matrix;
  stereo_calibration.left_distortion_params = camera_distortion_params;
  stereo_calibration.right_camera_matrix = projector_camera_matrix;
  stereo_calibration.right_distortion_params = projector_distorion_params;
  stereo_calibration.reprojection_error =
      stereo_calibration.reprojection_error = cv::stereoCalibrate(
          projector_object_points, image_points, image_points,
          stereo_calibration.left_camera_matrix,
          stereo_calibration.left_distortion_params,
          stereo_calibration.right_camera_matrix,
          stereo_calibration.right_distortion_params, image_size,
          stereo_calibration.R, stereo_calibration.T, stereo_calibration.E,
          stereo_calibration.F, cv::CALIB_FIX_INTRINSIC);

  return sfx::ConvertStereoCalibrationToProto(stereo_calibration);
}

absl::StatusOr<std::vector<std::vector<cv::Point3f>>> BackProject(
    const cv::Mat& camera_matrix, const cv::Mat& distortion_params,
    const cv::Mat& R, const cv::Mat& T,
    const std::vector<std::vector<cv::Point2f>>& image_points) {
  auto object_points = std::vector<std::vector<cv::Point3f>>(
      image_points.size(), std::vector<cv::Point3f>(image_points[0].size()));

  auto make_homography = [&](int32_t index) {
    cv::Mat homgraphy(3, 3, cv::DataType<double>::type);
    cv::Rodrigues(R.row(index).t(), homgraphy);
    homgraphy.at<double>(0, 2) = T.at<double>(index, 0);
    homgraphy.at<double>(1, 2) = T.at<double>(index, 1);
    homgraphy.at<double>(2, 2) = T.at<double>(index, 2);
    return homgraphy;
  };

  auto normalize = [](const cv::Mat& x) {
    CHECK_EQ(x.rows, 3);
    CHECK_EQ(x.cols, 1);
    CHECK_GT(x.at<double>(2, 0), 0);
    return cv::Mat({3, 1}, {x.at<double>(0, 0) / x.at<double>(2, 0),
                            x.at<double>(1, 0) / x.at<double>(2, 0), 0.0});
  };

  // TODO: Use distortion_param to re-map image points before back projecting.
  for (size_t i = 0; i < image_points.size(); ++i) {
    auto homography = make_homography(i);
    for (size_t j = 0; j < image_points[i].size(); ++j) {
      const cv::Mat image_point = cv::Mat_<double>(
          {3, 1}, {image_points[i][j].x, image_points[i][j].y, 1});
      const cv::Mat object_point =
          (camera_matrix * homography).inv() * image_point;
      const cv::Mat normalized_object_point = normalize(object_point);
      object_points[i][j] =
          cv::Point3f(normalized_object_point.at<double>(0, 0),
                      normalized_object_point.at<double>(1, 0),
                      normalized_object_point.at<double>(2, 0));
    }
  }
  return object_points;
}

}  // namespace sfx
