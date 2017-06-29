#include <math.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "map.hpp"

namespace cps2 {

Map::Map(float _grid_size, cps2::ImageEvaluator *_image_evaluator)
    : bbox(0, 0, _grid_size, _grid_size),
      grid_size(_grid_size),
      ready(false),
      image_evaluator(_image_evaluator),
      path_now(cv::Point3f(0, 0, 0) ),
      path_prev(cv::Point3f(0, 0, 0) )
{
  std::vector<MapPiece> v;
  v.push_back(MapPiece() );
  grid.push_back(v);
}

Map::~Map() {

}

cv::Point3f Map::image_distance(cv::Mat img1, cv::Mat img2, cv::Point3f flow_est) {
  // TODO change (at least flow_est.z and flow_corr) to radians

  // TODO get rid of magic numbers

  // TODO don't copy images: use references & instead

  // TODO assume incoming images as CV_8UC1 (gray)

  // flow_est in Bild-Koordinaten von img1 nach img2: x- Pixel nach rechts, y-Pixel nach oben theta- Drehung im Uhrzeigersinn
  int x = (int)flow_est.x;
  int y = (int)flow_est.y;
  int theta = (int)flow_est.z;

  cv::Mat gray1, gray2;
  cv::cvtColor(img1, gray1, CV_BGR2GRAY);
  cv::cvtColor(img2, gray2, CV_BGR2GRAY);

  //cut fisheye projection
  cv::Mat i1(gray1, cv::Rect(34, 56, img1.cols-68, img1.rows-112));
  cv::Mat i2(gray2, cv::Rect(34, 56, img1.cols-68, img1.rows-112));

  cv::Point3f flow_corr = flow_est;
  float min_error = 1000;

  for (int dtheta=-5; dtheta < 6; ++dtheta)
  {
    cv::Mat r = cv::getRotationMatrix2D(cv::Point2f(i2.cols/2., i2.rows/2.), theta+dtheta, 1.0);
    cv::Mat img2_rot;
    cv::warpAffine(i2, img2_rot, r, cv::Size(i2.cols, i2.rows));

    for (int dx=-10; dx < 11; ++dx)
    {
      for (int dy=-10; dy < 11; ++dy)
      {
        cv::Mat part1(i1, cv::Rect(std::max(x+dx, 0), std::max(y+dy, 0), i1.cols-abs(x+dx), i1.rows-abs(y+dy)));
        cv::Mat part2(img2_rot, cv::Rect(std::max(-x-dx, 0), std::max(-y-dy, 0), i1.cols-abs(x+dx), i1.rows-abs(y+dy)));

        cv::Mat tile1, tile2;
        part1.copyTo(tile1, part2);
        part2.copyTo(tile2, part1);

        cv::Mat test = tile1 - tile2;
        if (min_error > cv::sum(cv::mean(test))[0])
        {
          min_error = cv::sum(cv::mean(test))[0];
          flow_corr = cv::Point3f(x+dx, y+dy, theta+dtheta);
        }
      }
    }
  }

  return flow_corr;
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
  cv::Point2i pos_grid = world2grid(pos_world.p);
  MapPiece *map_piece  = &(grid.at(pos_grid.y).at(pos_grid.x) );
  cv::Point3f center   = grid2world(pos_grid.x, pos_grid.y);

  if(path_now != center) {
    path_prev = path_now;
    path_now  = center;
  }

  // update the mappiece if needed
  if(
      !map_piece->is_set
      || dist(pos_world.p, center) < dist(map_piece->pos_world, center)
      // TODO other criteria
  ) {
    map_piece->img    = image;
    map_piece->is_set = true;

    // TODO correct the position. Maybe like this:
    /*if(path_prev != path_now) {
      cv::Point2i path_prev_grid = world2grid(path_prev);

      map_piece->pos_world = pos_world.p + image_distance(
          grid.at(path_prev_grid.y).at(path_prev_grid.x).img,
          image, pos_world.p - grid.at(path_prev_grid.y).at(path_prev_grid.x).pos_world);
    }
    else*/
      map_piece->pos_world = pos_world.p;

    // TODO add timestamp
    // map_piece->stamp =
  }

  ready = true;
}

inline cv::Point2i Map::world2grid(const cv::Point3f &pos_world) {
  return cv::Point2i(
      (int)floorf( (pos_world.x - bbox.x) / grid_size),
      (int)floorf( (pos_world.y - bbox.y) / grid_size)
  );
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
