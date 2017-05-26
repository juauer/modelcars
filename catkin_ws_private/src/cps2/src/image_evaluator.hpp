#ifndef SRC_IMAGE_EVALUATOR_HPP_
#define SRC_IMAGE_EVALUATOR_HPP_

#include <opencv2/core.hpp>

#include "map.hpp"

namespace cps2 {

class ImageEvaluator {
 public:
  ImageEvaluator(Map &map, int resize_scale, int kernel_size, float kernel_stddev);
  ImageEvaluator(Map &map);
  virtual ~ImageEvaluator();

  float evaluateDummy(cv::Mat &img, cv::Point3f &particle);
  float evaluate(cv::Mat &img, cv::Point3f &particle);

 private:
  void generateKernel();
  int applyKernel(cv::Mat &img, int x, int y);

  Map map;
  cv::Mat kernel;
  int resize_scale;
  int kernel_size;
  float kernel_stddev;
};

} /* namespace cps2 */

#endif /* SRC_IMAGE_EVALUATOR_HPP_ */
