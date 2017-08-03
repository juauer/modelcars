#ifndef SRC_MAP_HPP_
#define SRC_MAP_HPP_

#include <vector>
#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include "fisheye_camera_matrix/camera_matrix.hpp"
#include "particle.hpp"
#include "image_evaluator.hpp"
#include "map_piece.hpp"

namespace cps2 {

class Map {
public:
  Map(cps2::ImageEvaluator *image_evaluator, bool is_big_map,
      float grid_size, float update_interval_min, float update_interval_max);

  virtual ~Map();

  /**
   * Compute vector between the centers of two images in world frame
   * @param img1 an image
   * @param img2 another image
   * @param pos_prev position of img1 in world frame
   * @param pos_new guessed position of img2 in world frame
   * @return corrected guess of position of img2 in world frame
   */
  cv::Point3f image_distance(const cv::Mat &img1, const cv::Mat &img2, const cv::Point3f &pos_prev, const cv::Point3f &pos_now);

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
  std::vector<std::vector<MapPiece> > grid;

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
   * Euclidean distance between p1 and p2. For low-d vectors this is much faster
   * than cv::norm or others
   * @param p1 point 1
   * @param p2 point 2
   * @return euclidean distance between p1 and p2
   */
  inline float dist(const cv::Point3f &p1, const cv::Point3f &p2);
  
  /**
   * Translate and rotate an image, truncating the borders.
   *
   * @param img
   * @param dx
   * @param dy
   * @param rotation
   * @return
   */
  cv::Mat transform(const cv::Mat &img, const int dx, const int dy, const float rotation);
  
  const float grid_size;
  const float update_interval_min;
  const float update_interval_max;

  bool ready;
  cps2::ImageEvaluator *image_evaluator;
  fisheye_camera_matrix::CameraMatrix camera_matrix;
  cv::Point3f path_now;
  cv::Point3f path_prev;

  // stuff for the (not yet obsolete?) big map
  bool is_big_map;
  cv::Mat big_map;
  cv::Point2i dim_img;
  cv::Point2i dim_map;
};

} /* namespace cps2 */

#endif /* SRC_MAP_HPP_ */
