#include "calibration/proto_utils.h"
#include "calibration/intrinsic_calibration.pb.h"
#include "opencv2/opencv.hpp"
// #include "calibration/stereo_calibration.h"
#include "calibration/stereo_calibration.pb.h"

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

proto::StereoCalibration ConvertStereoCalibrationToProto(
    const StereoCalibration& stereo_calibration) {
  proto::StereoCalibration result;

  result.mutable_left_camera_matrix()->set_fx(
      stereo_calibration.left_camera_matrix.at<double>(0, 0));
  result.mutable_left_camera_matrix()->set_fy(
      stereo_calibration.left_camera_matrix.at<double>(1, 1));
  result.mutable_left_camera_matrix()->set_cx(
      stereo_calibration.left_camera_matrix.at<double>(0, 2));
  result.mutable_left_camera_matrix()->set_cy(
      stereo_calibration.left_camera_matrix.at<double>(1, 2));

  result.mutable_left_camera_distortion()->set_k1(
      stereo_calibration.left_distortion_params.at<double>(0, 0));
  result.mutable_left_camera_distortion()->set_k2(
      stereo_calibration.left_distortion_params.at<double>(0, 1));
  result.mutable_left_camera_distortion()->set_k3(
      stereo_calibration.left_distortion_params.at<double>(0, 2));
  if (stereo_calibration.left_distortion_params.cols >= 4) {
    result.mutable_left_camera_distortion()->set_k4(
        stereo_calibration.left_distortion_params.at<double>(0, 3));
  }
  if (stereo_calibration.left_distortion_params.cols >= 5) {
    result.mutable_left_camera_distortion()->set_k5(
        stereo_calibration.left_distortion_params.at<double>(0, 4));
  }
  if (stereo_calibration.left_distortion_params.cols >= 6) {
    result.mutable_left_camera_distortion()->set_p1(
        stereo_calibration.left_distortion_params.at<double>(0, 5));
  }
  if (stereo_calibration.left_distortion_params.cols >= 7) {
    result.mutable_left_camera_distortion()->set_p2(
        stereo_calibration.left_distortion_params.at<double>(0, 6));
  }

  result.mutable_right_camera_matrix()->set_fx(
      stereo_calibration.right_camera_matrix.at<double>(0, 0));
  result.mutable_right_camera_matrix()->set_fy(
      stereo_calibration.right_camera_matrix.at<double>(1, 1));
  result.mutable_right_camera_matrix()->set_cx(
      stereo_calibration.right_camera_matrix.at<double>(0, 2));
  result.mutable_right_camera_matrix()->set_cy(
      stereo_calibration.right_camera_matrix.at<double>(1, 2));

  result.mutable_right_camera_distortion()->set_k1(
      stereo_calibration.right_distortion_params.at<double>(0, 0));
  result.mutable_right_camera_distortion()->set_k2(
      stereo_calibration.right_distortion_params.at<double>(0, 1));
  result.mutable_right_camera_distortion()->set_k3(
      stereo_calibration.right_distortion_params.at<double>(0, 2));
  if (stereo_calibration.right_distortion_params.cols >= 4) {
    result.mutable_right_camera_distortion()->set_k4(
        stereo_calibration.right_distortion_params.at<double>(0, 3));
  }
  if (stereo_calibration.right_distortion_params.cols >= 5) {
    result.mutable_right_camera_distortion()->set_k5(
        stereo_calibration.right_distortion_params.at<double>(0, 4));
  }
  // Ignore p1 and p2 because cv::StereoCalibration returns 1x7 shape with zero
  // p1 and p2

  result.mutable_r()->set_r11(stereo_calibration.R.at<double>(0, 0));
  result.mutable_r()->set_r12(stereo_calibration.R.at<double>(0, 1));
  result.mutable_r()->set_r13(stereo_calibration.R.at<double>(0, 2));
  result.mutable_r()->set_r21(stereo_calibration.R.at<double>(1, 0));
  result.mutable_r()->set_r22(stereo_calibration.R.at<double>(1, 1));
  result.mutable_r()->set_r23(stereo_calibration.R.at<double>(1, 2));
  result.mutable_r()->set_r31(stereo_calibration.R.at<double>(2, 0));
  result.mutable_r()->set_r32(stereo_calibration.R.at<double>(2, 1));
  result.mutable_r()->set_r33(stereo_calibration.R.at<double>(2, 2));

  result.mutable_t()->set_r11(stereo_calibration.T.at<double>(0, 0));
  result.mutable_t()->set_r21(stereo_calibration.T.at<double>(1, 0));
  result.mutable_t()->set_r31(stereo_calibration.T.at<double>(2, 0));

  result.mutable_e()->set_r11(stereo_calibration.E.at<double>(0, 0));
  result.mutable_e()->set_r12(stereo_calibration.E.at<double>(0, 1));
  result.mutable_e()->set_r13(stereo_calibration.E.at<double>(0, 2));
  result.mutable_e()->set_r21(stereo_calibration.E.at<double>(1, 0));
  result.mutable_e()->set_r22(stereo_calibration.E.at<double>(1, 1));
  result.mutable_e()->set_r23(stereo_calibration.E.at<double>(1, 2));
  result.mutable_e()->set_r31(stereo_calibration.E.at<double>(2, 0));
  result.mutable_e()->set_r32(stereo_calibration.E.at<double>(2, 1));
  result.mutable_e()->set_r33(stereo_calibration.E.at<double>(2, 2));

  result.mutable_f()->set_r11(stereo_calibration.F.at<double>(0, 0));
  result.mutable_f()->set_r12(stereo_calibration.F.at<double>(0, 1));
  result.mutable_f()->set_r13(stereo_calibration.F.at<double>(0, 2));
  result.mutable_f()->set_r21(stereo_calibration.F.at<double>(1, 0));
  result.mutable_f()->set_r22(stereo_calibration.F.at<double>(1, 1));
  result.mutable_f()->set_r23(stereo_calibration.F.at<double>(1, 2));
  result.mutable_f()->set_r31(stereo_calibration.F.at<double>(2, 0));
  result.mutable_f()->set_r32(stereo_calibration.F.at<double>(2, 1));
  result.mutable_f()->set_r33(stereo_calibration.F.at<double>(2, 2));

  result.set_reprojection_error(stereo_calibration.reprojection_error);

  return result;
}

StereoCalibration ConvertStereoCalibrationFromProto(
    const proto::StereoCalibration& stereo_calibration) {
  StereoCalibration result;

  result.left_camera_matrix = cv::Mat::eye(3, 3, CV_64F);
  result.left_camera_matrix.at<double>(0, 0) =
      stereo_calibration.left_camera_matrix().fx();
  result.left_camera_matrix.at<double>(1, 1) =
      stereo_calibration.left_camera_matrix().fy();
  result.left_camera_matrix.at<double>(0, 2) =
      stereo_calibration.left_camera_matrix().cx();
  result.left_camera_matrix.at<double>(1, 2) =
      stereo_calibration.left_camera_matrix().cy();

  result.right_camera_matrix = cv::Mat::eye(3, 3, CV_64F);
  result.right_camera_matrix.at<double>(0, 0) =
      stereo_calibration.right_camera_matrix().fx();
  result.right_camera_matrix.at<double>(1, 1) =
      stereo_calibration.right_camera_matrix().fy();
  result.right_camera_matrix.at<double>(0, 2) =
      stereo_calibration.right_camera_matrix().cx();
  result.right_camera_matrix.at<double>(1, 2) =
      stereo_calibration.right_camera_matrix().cy();

  // Ignore p1 and p2
  constexpr int32_t kNumDistortionParams = 5;
  result.left_distortion_params =
      cv::Mat::zeros(1, kNumDistortionParams, CV_64F);
  result.left_distortion_params.at<double>(0, 0) =
      stereo_calibration.left_camera_distortion().k1();
  result.left_distortion_params.at<double>(0, 1) =
      stereo_calibration.left_camera_distortion().k2();
  result.left_distortion_params.at<double>(0, 2) =
      stereo_calibration.left_camera_distortion().k3();
  if (stereo_calibration.left_camera_distortion().has_k4()) {
    result.left_distortion_params.at<double>(0, 3) =
        stereo_calibration.left_camera_distortion().k4();
  }
  if (stereo_calibration.left_camera_distortion().has_k5()) {
    result.left_distortion_params.at<double>(0, 4) =
        stereo_calibration.left_camera_distortion().k5();
  }

  result.right_distortion_params =
      cv::Mat::zeros(1, kNumDistortionParams, CV_64F);
  result.right_distortion_params.at<double>(0, 0) =
      stereo_calibration.right_camera_distortion().k1();
  result.right_distortion_params.at<double>(0, 1) =
      stereo_calibration.right_camera_distortion().k2();
  result.right_distortion_params.at<double>(0, 2) =
      stereo_calibration.right_camera_distortion().k3();
  if (stereo_calibration.right_camera_distortion().has_k4()) {
    result.right_distortion_params.at<double>(0, 3) =
        stereo_calibration.right_camera_distortion().k4();
  }
  if (stereo_calibration.right_camera_distortion().has_k5()) {
    result.right_distortion_params.at<double>(0, 4) =
        stereo_calibration.right_camera_distortion().k5();
  }
  if (stereo_calibration.right_camera_distortion().has_p1()) {
    result.right_distortion_params.at<double>(0, 5) =
        stereo_calibration.right_camera_distortion().p1();
  }
  if (stereo_calibration.right_camera_distortion().has_p2()) {
    result.right_distortion_params.at<double>(0, 6) =
        stereo_calibration.right_camera_distortion().p2();
  }

  result.R = cv::Mat::eye(3, 3, CV_64F);
  result.R.at<double>(0, 0) = stereo_calibration.r().r11();
  result.R.at<double>(0, 1) = stereo_calibration.r().r12();
  result.R.at<double>(0, 2) = stereo_calibration.r().r13();
  result.R.at<double>(1, 0) = stereo_calibration.r().r21();
  result.R.at<double>(1, 1) = stereo_calibration.r().r22();
  result.R.at<double>(1, 2) = stereo_calibration.r().r23();
  result.R.at<double>(2, 0) = stereo_calibration.r().r31();
  result.R.at<double>(2, 1) = stereo_calibration.r().r32();
  result.R.at<double>(2, 2) = stereo_calibration.r().r33();

  result.T = cv::Mat::zeros(3, 1, CV_64F);
  result.T.at<double>(0, 0) = stereo_calibration.t().r11();
  result.T.at<double>(1, 0) = stereo_calibration.t().r21();
  result.T.at<double>(2, 0) = stereo_calibration.t().r31();

  result.E = cv::Mat::eye(3, 3, CV_64F);
  result.E.at<double>(0, 0) = stereo_calibration.e().r11();
  result.E.at<double>(0, 1) = stereo_calibration.e().r12();
  result.E.at<double>(0, 2) = stereo_calibration.e().r13();
  result.E.at<double>(1, 0) = stereo_calibration.e().r21();
  result.E.at<double>(1, 1) = stereo_calibration.e().r22();
  result.E.at<double>(1, 2) = stereo_calibration.e().r23();
  result.E.at<double>(2, 0) = stereo_calibration.e().r31();
  result.E.at<double>(2, 1) = stereo_calibration.e().r32();
  result.E.at<double>(2, 2) = stereo_calibration.e().r33();

  result.F = cv::Mat::eye(3, 3, CV_64F);
  result.F.at<double>(0, 0) = stereo_calibration.f().r11();
  result.F.at<double>(0, 1) = stereo_calibration.f().r12();
  result.F.at<double>(0, 2) = stereo_calibration.f().r13();
  result.F.at<double>(1, 0) = stereo_calibration.f().r21();
  result.F.at<double>(1, 1) = stereo_calibration.f().r22();
  result.F.at<double>(1, 2) = stereo_calibration.f().r23();
  result.F.at<double>(2, 0) = stereo_calibration.f().r31();
  result.F.at<double>(2, 1) = stereo_calibration.f().r32();
  result.F.at<double>(2, 2) = stereo_calibration.f().r33();

  result.reprojection_error = stereo_calibration.reprojection_error();

  return result;
}

}  // namespace sfx
