#include "graycode.h"
#include "gmock/gmock-matchers.h"
#include "gtest/gtest.h"

namespace sfx {
namespace {

using ::testing::Pointwise;

MATCHER(MatExactlyEq, "two cv::Mat values are exactly equal") {
  const cv::Mat& actual = std::get<0>(arg);
  const cv::Mat& expected = std::get<1>(arg);
  if (actual.rows != expected.rows || actual.cols != expected.cols ||
      actual.type() != expected.type()) {
    return false;
  }
  return cv::norm(actual, expected, cv::NORM_INF) == 0.0;
}

TEST(GrayCode, SmokeTest) {
  GrayCode gray_code(3, 2);
  const std::vector<cv::Mat> expected = {
      (cv::Mat_<uint8_t>(2, 3) << 255, 255, 255, 255, 255, 255),
      (cv::Mat_<uint8_t>(2, 3) << 0, 0, 0, 0, 0, 0),
      (cv::Mat_<uint8_t>(2, 3) << 0, 0, 255, 0, 0, 255),
      (cv::Mat_<uint8_t>(2, 3) << 255, 255, 0, 255, 255, 0),
      (cv::Mat_<uint8_t>(2, 3) << 0, 255, 255, 0, 255, 255),
      (cv::Mat_<uint8_t>(2, 3) << 255, 0, 0, 255, 0, 0),
      (cv::Mat_<uint8_t>(2, 3) << 0, 0, 0, 255, 255, 255),
      (cv::Mat_<uint8_t>(2, 3) << 255, 255, 255, 0, 0, 0),
  };
  EXPECT_THAT(gray_code.Generate(), Pointwise(MatExactlyEq(), expected));
}

}  // namespace
}  // namespace sfx