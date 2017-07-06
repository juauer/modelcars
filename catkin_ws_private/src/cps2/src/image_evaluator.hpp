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

  /**
   * Compute a rotated subimage and apply downscaling and blurring during the process.
   * @param img an grayscale image
   * @param pos_image center of the new image, relative to the original image origin
   * @param th rotate the resulting image around this angle
   * @param ph rotate the original image around this angle
   * @return the transformed image
   */
  cv::Mat transform(const cv::Mat &img,
      const cv::Point2i &pos_image, const float th, const float ph);

  float evaluate(const cv::Mat &img1, const cv::Mat &img2);

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
