#include "graycode.h"
#include <cmath>

namespace sfx {

constexpr u_int8_t kWhite = 255;
constexpr u_int8_t kBlack = 0;

GrayCode::GrayCode(std::size_t width, std::size_t height)
    : width_(width), height_(height) {
  col_bits_ = std::ceil(std::log2(width_));
  row_bits_ = std::ceil(std::log2(height_));
  gray_codes_.resize(col_bits_ * 2 + row_bits_ * 2 + 2);
  for (size_t i = 0; i < gray_codes_.size(); ++i) {
    gray_codes_[i] = cv::Mat(height_, width_, CV_8UC1, cv::Scalar(kBlack));
  }
  gray_codes_[0] = cv::Mat(height_, width_, CV_8UC1, cv::Scalar(kWhite));
}

const std::vector<cv::Mat>& GrayCode::Generate() {
  uint8_t flag = 0;
  // Column patterns
  for (size_t j = 0; j < width_; ++j) {
    int32_t rem = 0;
    int32_t num = j;
    int32_t prev_rem = j % 2;

    for (size_t k = 0; k < col_bits_; ++k) {
      num = num / 2;
      rem = num % 2;
      if ((rem == 0 && prev_rem == 1) || (rem == 1 && prev_rem == 0)) {
        flag = 1;
      } else {
        flag = 0;
      }
      for (size_t i = 0; i < height_; ++i) {
        uint8_t pixel_color = flag * kWhite;
        gray_codes_[2 * col_bits_ - 2 * k].at<uint8_t>(i, j) = pixel_color;
        pixel_color = pixel_color == 0 ? kWhite : kBlack;
        gray_codes_[2 * col_bits_ - 2 * k + 1].at<u_int8_t>(i, j) = pixel_color;
      }
      prev_rem = rem;
    }
  }

  // Row patterns
  for (size_t i = 0; i < height_; ++i) {
    int32_t rem = 0;
    int32_t num = i;
    int32_t prev_rem = i % 2;
    // For each bit in the row
    for (size_t k = 0; k < row_bits_; k++) {
      num = num / 2;
      rem = num % 2;
      if ((rem == 0 && prev_rem == 1) || (rem == 1 && prev_rem == 0)) {
        flag = 1;
      } else {
        flag = 0;
      }

      for (size_t j = 0; j < width_; ++j) {
        uint8_t pixel_color = flag * kWhite;
        gray_codes_[2 * row_bits_ - 2 * k + 2 * col_bits_].at<u_int8_t>(i, j) =
            pixel_color;
        pixel_color = pixel_color == 0 ? kWhite : kBlack;
        gray_codes_[2 * row_bits_ - 2 * k + 2 * col_bits_ + 1].at<uint8_t>(i, j) =
            pixel_color;
      }
      prev_rem = rem;
    }
  }
  return gray_codes_;
}

}  // namespace sfx
