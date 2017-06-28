#include <math.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "map.hpp"

namespace cps2 {

Map::Map(float _grid_size, cps2::ImageEvaluator *_image_evaluator)
    : bbox(0, 0, _grid_size, _grid_size),
      grid_size(_grid_size),
      ready(false),
      image_evaluator(_image_evaluator)
{
  std::vector<MapPiece> v;
  v.push_back(MapPiece() );
  grid.push_back(v);
}

Map::~Map() {

}

std::vector<cv::Mat> Map::get_map_pieces(const cv::Point3f &pos_world) {
  std::vector<cv::Mat> map_pieces;

  if(!ready)
    return map_pieces;

  // TODO locate and push_back several images near pos_world

  cv::Point2f pos_rel2f(pos_world.x, pos_world.y);

  cv::Point2i pos_image = camera_matrix.relative2image(pos_rel2f);

  cv::Point3f pos_image3f(pos_image.x, pos_image.y, pos_world.z);

  map_pieces.push_back(image_evaluator->transform(
      theOnePiece.img, pos_image3f, cv::Size2i(camera_matrix.width, camera_matrix.height) ) );

  return map_pieces;
}

void Map::update(const cv::Mat &image, const Particle &pos_world,
      const fisheye_camera_matrix::CameraMatrix &_camera_matrix) {
  // update camera_matrix (with respect to auto-calibration, dynamic height, etc.)
  camera_matrix = _camera_matrix;

  // resize the grid if needed
  if(pos_world.p.x - grid_size < bbox.x) {
    for(std::vector<std::vector<MapPiece> >::iterator y = grid.begin(); y != grid.end(); ++y)
      y->insert(y->begin(), MapPiece() );

    bbox.x     -= grid_size;
    bbox.width += grid_size;
  }
  else if(pos_world.p.y - grid_size < bbox.y) {
    std::vector<MapPiece> v(bbox.width / grid_size);

    grid.insert(grid.begin(), v);

    bbox.y      -= grid_size;
    bbox.height += grid_size;
  }
  else if(pos_world.p.x  + grid_size > bbox.x + bbox.width) {
    for(std::vector<std::vector<MapPiece> >::iterator y = grid.begin(); y != grid.end(); ++y)
      y->push_back(MapPiece() );

    bbox.width += grid_size;
  }
  else if(pos_world.p.y  + grid_size > bbox.y + bbox.height) {
    std::vector<MapPiece> v(bbox.width / grid_size);

    grid.push_back(v);

    bbox.height += grid_size;
  }

  // get the mappiece for pos_world
  int grid_x, grid_y;

  world2grid(pos_world.p, grid_x, grid_y);

  MapPiece *map_piece = &(grid.at(grid_y).at(grid_x));
  cv::Point3f center  = grid2world(grid_x, grid_y);

  // update the mappiece if needed
  if(
      !map_piece->is_set
      || dist(pos_world.p, center) < dist(map_piece->pos_world, center)
      // TODO other criteria
  ) {
    map_piece->img       = image;
    map_piece->is_set    = true;

    // TODO correct the position
    map_piece->pos_world = pos_world.p;

    // TODO add timestamp
    // map_piece->stamp =
  }

  ready = true;
}

inline void Map::world2grid(const cv::Point3f &pos_world, int &grid_x, int &grid_y) {
  grid_x = (int)floorf( (pos_world.x - bbox.x) / grid_size);
  grid_y = (int)floorf( (pos_world.y - bbox.y) / grid_size);
}


inline cv::Point3f Map::grid2world(const int &grid_x, const int &grid_y) {
  return cv::Point3f(
      (grid_x + 0.5) * grid_size + bbox.x,
      (grid_y + 0.5) * grid_size + bbox.y,
      0
  );
}

cv::Point2f Map::rotate(const cv::Point2f &p, const float th) {
  const float ths = sinf(th);
  const float thc = cosf(th);

  return cv::Point2f(
      thc * p.x - ths * p.y,
      ths * p.x + thc * p.y
  );
}

inline float Map::dist(const cv::Point3f &p1, const cv::Point3f &p2) {
  const float x = p1.x - p2.x;
  const float y = p1.y - p2.y;

  return sqrtf(x * x + y * y);
}

}
