#ifndef SRC_MAP_HPP_
#define SRC_MAP_HPP_

#include <opencv2/core.hpp>

namespace cps2 {

class Map {
public:
	Map();
	virtual ~Map();

	cv::Mat img;
};

} /* namespace cps2 */

#endif /* SRC_MAP_HPP_ */
