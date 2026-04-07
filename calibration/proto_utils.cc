#include "calibration/proto_utils.h"
#include "opencv2/opencv.hpp"

namespace sfx {

proto::IntrinsicCalibration ConvertIntrinsicCalibrationToProto(
    const IntrinsicCalibration& intrinsic_calibration) {
  proto::IntrinsicCalibration result;
  result.mutable_camera_matrix()->set_fx(
      intrinsic_calibration.camera_matrix.at<double>(0, 0));
  result.mutable_camera_matrix()->set_cx(
      intrinsic_calibration.camera_matrix.at<double>(0, 2));
  result.mutable_camera_matrix()->set_fy(
      intrinsic_calibration.camera_matrix.at<double>(1, 1));
  result.mutable_camera_matrix()->set_cy(
      intrinsic_calibration.camera_matrix.at<double>(1, 2));
  result.mutable_distortion_parameters()->set_k1(
      intrinsic_calibration.distortion_params.at<double>(0, 0));
  result.mutable_distortion_parameters()->set_k2(
      intrinsic_calibration.distortion_params.at<double>(0, 1));
  result.mutable_distortion_parameters()->set_k3(
      intrinsic_calibration.distortion_params.at<double>(0, 2));
  result.mutable_distortion_parameters()->set_k4(
      intrinsic_calibration.distortion_params.at<double>(0, 3));
  if (intrinsic_calibration.distortion_params.cols >= 5) {
    result.mutable_distortion_parameters()->set_k5(
      intrinsic_calibration.distortion_params.at<double>(0, 4));
  }
  if (intrinsic_calibration.distortion_params.cols >= 6) {
    result.mutable_distortion_parameters()->set_p1(
      intrinsic_calibration.distortion_params.at<double>(0, 5));
  }
  if (intrinsic_calibration.distortion_params.cols >= 7) {
    result.mutable_distortion_parameters()->set_p2(
      intrinsic_calibration.distortion_params.at<double>(0, 6));
  }
  result.set_reprojection_error(intrinsic_calibration.reprojection_error);
  return result;
}

IntrinsicCalibration ConvertIntrinsicCalibrationFromProto(
    const proto::IntrinsicCalibration& proto) {
  IntrinsicCalibration result;
  result.camera_matrix = cv::Mat::eye(3, 3, CV_64F);
  result.camera_matrix.at<double>(0, 0) = proto.camera_matrix().fx();
  result.camera_matrix.at<double>(1, 1) = proto.camera_matrix().fy();
  result.camera_matrix.at<double>(0, 2) = proto.camera_matrix().cx();
  result.camera_matrix.at<double>(1, 2) = proto.camera_matrix().cy();
  result.reprojection_error = proto.reprojection_error();
  return result;
}

}  // namespace sfx
