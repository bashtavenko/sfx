// https://en.wikipedia.org/wiki/Gray_code
// https://3dunderworld.org/
#ifndef SFX_GRAYCODE_H
#define SFX_GRAYCODE_H

#include <vector>
#include "opencv2/opencv.hpp"

namespace sfx {

class GrayCode {
 public:
  explicit GrayCode(size_t width, size_t height);
  const std::vector<cv::Mat>& Generate();

 private:
  size_t width_;
  size_t height_;
  size_t col_bits_;
  size_t row_bits_;
  std::vector<cv::Mat> gray_codes_;
};

}  // namespace sfx

#endif  // SFX_GRAYCODE_H
