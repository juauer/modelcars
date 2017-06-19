#ifndef SRC_IMAGE_EVALUATOR_HPP_
#define SRC_IMAGE_EVALUATOR_HPP_

#include <opencv2/core.hpp>

namespace cps2 {

const int IE_MODE_PIXELS    = 0;
const int IE_MODE_CENTROIDS = 1;

class ImageEvaluator {
 public:
  ImageEvaluator(int mode, int resize_scale, int kernel_size, float kernel_stddev);
  virtual ~ImageEvaluator();

  cv::Mat transform(const cv::Mat &img, const cv::Point3f &pos_image, cv::Size onePieceMapSizeHACK);
  float evaluate(cv::Mat &img1, cv::Mat &img2);

 private:
  void generateKernel();
  int applyKernel(const cv::Mat &img, int x, int y);

  cv::Mat kernel;
  int mode;
  int resize_scale;
  int kernel_size;
  float kernel_stddev;
};

} /* namespace cps2 */

#endif /* SRC_IMAGE_EVALUATOR_HPP_ */
