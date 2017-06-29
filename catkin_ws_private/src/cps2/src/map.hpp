#ifndef SRC_MAP_HPP_
#define SRC_MAP_HPP_

#include <vector>
#include <ros/ros.h>
#include <opencv2/core.hpp>
#include "fisheye_camera_matrix/camera_matrix.hpp"
#include "particle.hpp"
#include "image_evaluator.hpp"
#include "map_piece.hpp"

namespace cps2 {

class Map {
public:
  Map(cps2::ImageEvaluator *image_evaluator, float grid_size,
      float update_interval_min, float update_interval_max);

  virtual ~Map();

  static cv::Point3f image_distance(cv::Mat img1, cv::Mat img2, cv::Point3f flow_est);

  /**
   * Get one or more images, which are centered at pos_world in world frame.
   * @param pos_world pose in world frame
   * @return list of images, which are centered at pos_world in world frame
   */
  std::vector<cv::Mat> get_map_pieces(const cv::Point3f &pos_world);

  /**
   * Update the map with crucial data. Should get called every frame. The map decides on
   * best effort if a update is needed and may return immediately.
   * @param image current undistorted, grayscale image
   * @param pos_world current best guess of the position in world frame
   * @param camera_matrix current camera matrix
   */
  void update(const cv::Mat &image, const Particle &pos_world,
      const fisheye_camera_matrix::CameraMatrix &camera_matrix);

  cv::Rect2f bbox; //!< Bounding box in world frame covering the yet mapped space

private:
  /**
   * Get grid indices of nearest grid cell for some world coordinates.
   * It's not guaranteed at any time the computed indices do exist. One should
   * always check against bbox.
   * @param pos_world input world coordinates
   * @param grid_x output grid index x
   * @param grid_y output grid index y
   */
  inline cv::Point2i world2grid(const cv::Point3f &pos_world);

  /**
   * Get center of a cell in world frame for given grid indices.
   * @param grid_x input grid index x
   * @param grid_y input grid index y
   * @return center of cell in world frame
   */
  inline cv::Point3f grid2world(const int &grid_x, const int &grid_y);

  /**
   * Rotate a vector around its origin
   * @param p vector
   * @param th angle in rad
   * @return rotated vector
   */
  inline cv::Point2f rotate(const cv::Point2f &p, const float th);

  /**
   * Euclidean distance between p1 and p2. For low-d vectors this is much faster
   * than cv::norm or others
   * @param p1 point 1
   * @param p2 point 2
   * @return euclidean distance between p1 and p2
   */
  inline float dist(const cv::Point3f &p1, const cv::Point3f &p2);

  const float grid_size;
  const float update_interval_min;
  const float update_interval_max;

  bool ready;
  cps2::ImageEvaluator *image_evaluator;
  fisheye_camera_matrix::CameraMatrix camera_matrix;
  std::vector<std::vector<MapPiece> > grid;
  cv::Point3f path_now;
  cv::Point3f path_prev;
};

} /* namespace cps2 */

#endif /* SRC_MAP_HPP_ */
