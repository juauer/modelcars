#include <math.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "map.hpp"

namespace cps2 {

Map::Map(const char *path, cps2::ImageEvaluator *_image_evaluator)
    : image_evaluator(_image_evaluator)
{
  ready = false;

  // TODO load a bunch of images, associated data and some index instead of a single file.

  cv::Mat img_bgr = cv::imread(path);
  cv::Mat img_gray;

  cv::cvtColor(img_bgr, img_gray, CV_BGR2GRAY);

  bbox        = cv::Rect2f(0, 0, 0, 0);
  origin      = cv::Point3f(0, 0, 0);
  theOnePiece = cps2::MapPiece(origin, img_gray);
}

Map::~Map() {

}

std::vector<cv::Mat> Map::get_map_pieces(const cv::Point3f &pos_world) {
  std::vector<cv::Mat> map_pieces;

  if(!ready)
    return map_pieces;

  // TODO locate and push_back several images near pos_world

  // TODO take origin into account

  cv::Point2f pos_rel2f(pos_world.x, pos_world.y);

  cv::Point2i pos_image = camera_matrix.relative2image(pos_rel2f);

  cv::Point3f pos_image3f(pos_image.x, pos_image.y,
      pos_world.z - theOnePiece.pos_world.z);

  map_pieces.push_back(image_evaluator->transform(
      theOnePiece.img, pos_image3f, cv::Size2i(camera_matrix.width, camera_matrix.height) ) );

  return map_pieces;
}

void Map::update(const cv::Point3f &pos_world_last, const cv::Point3f &pos_world_now,
      const cv::Point2f &pos_relative_vel, float belief_best,
      const fisheye_camera_matrix::CameraMatrix &_camera_matrix) {

  // TODO get rid of the onePieceMapHACK

  camera_matrix        = _camera_matrix;
  camera_matrix.width  = theOnePiece.img.cols;
  camera_matrix.height = theOnePiece.img.rows;
  camera_matrix.cx     = theOnePiece.img.cols / 2;
  camera_matrix.cy     = theOnePiece.img.rows / 2;

  // TODO update using the latest data

  cv::Point2i p_img(0, 0);

  cv::Point2f p_rel = camera_matrix.image2relative(p_img);
  cv::Point2f p_abs = cv::Point2f(fabs(p_rel.x), fabs(p_rel.y) );

  // TODO do something smart to handle the borders instead of using magic numbers

  bbox.x      = -0.9 * p_abs.x;
  bbox.y      = -0.9 * p_abs.y;
  bbox.width  = 1.8 * p_abs.x;
  bbox.height = 1.8 * p_abs.y;
  ready       = true;
}
}
