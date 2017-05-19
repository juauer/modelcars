#ifndef SRC_IMAGE_EVALUATOR_HPP_
#define SRC_IMAGE_EVALUATOR_HPP_

#include "map.hpp"

namespace cps2 {

class ImageEvaluator {
public:
	ImageEvaluator(Map &map);
	virtual ~ImageEvaluator();

	float evaluate(cv::Mat &img, cv::Point3f &particle);

	const int resize_scale = 3;

private:
	Map map;
};

} /* namespace cps2 */

#endif /* SRC_IMAGE_EVALUATOR_HPP_ */
