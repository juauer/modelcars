#ifndef SRC_MAP_HPP_
#define SRC_MAP_HPP_

#include <opencv2/core.hpp>

namespace cps2 {

class Map {
public:
	Map(const char *path);
	virtual ~Map();

	cv::Point3f map2world(cv::Point3f &pose_map);
	cv::Point3f world2map(cv::Point3f &pose_world);

	cv::Mat img_bgr;
	cv::Mat img_gray;
};

} /* namespace cps2 */

#endif /* SRC_MAP_HPP_ */
