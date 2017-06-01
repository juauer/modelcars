#ifndef SRC_IMAGE_EVALUATOR_HPP_
#define SRC_IMAGE_EVALUATOR_HPP_

#include <opencv2/core.hpp>

#include "map.hpp"

namespace cps2 {

const int IE_MODE_PIXELS    = 0;
const int IE_MODE_CENTROIDS = 1;

class ImageEvaluator {
 public:
  ImageEvaluator(Map &map, int mode, int resize_scale, int kernel_size, float kernel_stddev);
  ImageEvaluator(Map &map, int mode);
  virtual ~ImageEvaluator();

  float evaluate(cv::Mat &img, cv::Point3f &particle);

 private:
  void generateKernel();
  int applyKernel(cv::Mat &img, int x, int y);

  Map map;
  cv::Mat kernel;
  int mode;
  int resize_scale;
  int kernel_size;
  float kernel_stddev;
};

} /* namespace cps2 */

#endif /* SRC_IMAGE_EVALUATOR_HPP_ */
