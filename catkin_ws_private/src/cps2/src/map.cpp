#include <math.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/package.h>
#include "map.hpp"

namespace cps2 {

/*cv::Mat mmm;
cv::Point2i mc;*/

Map::Map(cps2::ImageEvaluator *_image_evaluator, float _grid_size,
    float _update_interval_min, float _update_interval_max)
    : grid_size(_grid_size),
      update_interval_min(_update_interval_min),
      update_interval_max(_update_interval_max),
      ready(false),
      bbox(0, 0, _grid_size, _grid_size),
      // bbox(-3, -3, 6, 6),
      image_evaluator(_image_evaluator),
      path_now(cv::Point3f(_grid_size / 2, _grid_size / 2, 0) ),
      path_prev(cv::Point3f(_grid_size / 2, _grid_size / 2, 0) )
{
  // start with a 1x1 grid
  std::vector<MapPiece> v;
  v.push_back(MapPiece() );
  grid.push_back(v);

  /*mmm = cv::imread("/home/juauer/Schreibtisch/modelcars/captures/map_test_2.png");
  cv::cvtColor(mmm, mmm, CV_BGR2GRAY);
  mc = cv::Point2i(mmm.cols/2, mmm.rows/2);*/
}

Map::~Map() {

}

std::vector<cv::Mat> Map::get_map_pieces(const cv::Point3f &pos_world) {
  std::vector<cv::Mat> map_piece_images;

  if(!ready)
    return map_piece_images;

  /*cv::Point2i p = camera_matrix.relative2image(cv::Point2f(pos_world.x, pos_world.y));
  cv::Mat mm = image_evaluator->transform(mmm, p + mc-cv::Point2i(320,240), pos_world.z, 0, 480, 640);
  map_piece_images.push_back(mm);
  return map_piece_images;*/

  // get grid indices and world coords of the grid cells center for pos_world
  const cv::Point2i pos_grid = world2grid(pos_world);
  const cv::Point3f center   = grid2world(pos_grid.x, pos_grid.y);

  // find up to two cells which contain a valid (set) mappiece that was recorded near pos_world
  MapPiece map_pieces[2] = { };

  const int grid_x_lb = std::max(0, pos_grid.x - 1);
  const int grid_x_ub = std::min( (int)grid.at(0).size(), pos_grid.x + 2);
  const int grid_y_lb = std::max(0, pos_grid.y - 1);
  const int grid_y_ub = std::min( (int)grid.size(), pos_grid.y + 2);

  for(int k = 0; k < 2; ++k) {
    // check a 3x3 grid for image with least distance
    for(int i = grid_y_lb; i < grid_y_ub; ++i)
      for(int j = grid_x_lb; j < grid_x_ub; ++j) {
        // do not use data from pos_grid itself, to avoid reading and updating the same mappiece
        if(i == pos_grid.y && j == pos_grid.x)
          continue;

        // skip unset pieces
        if(!grid.at(i).at(j).is_set)
          continue;

        // treat k==1
        if(k == 1 && map_pieces[0].pos_world.x == grid.at(i).at(j).pos_world.x
            && map_pieces[0].pos_world.y == grid.at(i).at(j).pos_world.y)
          continue;

        // always prefer set pieces over unset ones. Check the distance as final criterium
        if(!map_pieces[k].is_set
           || dist(grid.at(i).at(j).pos_world, center) < dist(map_pieces[k].pos_world, center)
        )
          map_pieces[k] = grid.at(i).at(j);
      }
  }

  // if there is no other option, use pos_grid itself
  if(!map_pieces[0].is_set && pos_grid.x > 0 && pos_grid.y > 0
      && pos_grid.x < grid.at(0).size() && pos_grid.y < grid.size() )
    map_pieces[0] = grid.at(pos_grid.y).at(pos_grid.x);

  // extract the images
  for(int k = 0; k < 2; ++k) {
    if(!map_pieces[k].is_set)
      break;

    // vector between pos_world and center of mappiece
    cv::Point2f pos_rel(
        pos_world.x - map_pieces[k].pos_world.x,
        pos_world.y - map_pieces[k].pos_world.y);

    // projection of pos_world to image plane
    cv::Point2i pos_image = camera_matrix.relative2image(pos_rel);

    // transformed image with respect to pos_world rotation and mappiece rotation
    map_piece_images.push_back(image_evaluator->transform(
        map_pieces[k].img, pos_image, pos_world.z, -map_pieces[k].pos_world.z) );
  }

  return map_piece_images;
}

cv::Point3f Map::image_distance(const cv::Mat &img1, const cv::Mat &img2,
      const cv::Point3f &pos_prev, const cv::Point3f &pos_now) {

  // this is a brute force experimental approach!
  const cv::Point2i shift = camera_matrix.relative2image(
      cv::Point2f(pos_now.x - pos_prev.x, pos_now.y - pos_prev.y) )
      - cv::Point2i(img1.cols / 2, img1.rows / 2);

  int best_x     = shift.x;
  int best_y     = shift.y;
  float best_th  = pos_now.z - pos_prev.z;
  float best_err = 1;

  for(float th = pos_now.z - pos_prev.z - 0.1; th <= pos_now.z - pos_prev.z + 0.1; th += 0.02)
    for(int dx = shift.x - 10; dx <= shift.x + 10; dx += 2)
      for(int dy = shift.y - 20; dy <= shift.y + 20; dy += 4) {
        const cv::Mat img1_cut = transform(img1,  dx / 2,  dy / 2, 0);
        const cv::Mat img2_cut = transform(img2, -dx / 2, -dy / 2, -th);
        const float err        = image_evaluator->evaluate(img1_cut, img2_cut);

        if(err < best_err) {
          best_x   = dx;
          best_y   = dy;
          best_th  = th;
          best_err = err;
        }
      }

#ifdef DEBUG_IMAGE_DISTANCE
  printf("correction in image frame      : %d, %d, %.2f\n",
        best_x - shift.x, best_y - shift.y, best_th - pos_now.z + pos_prev.z);

  cv::Mat img1_cut = transform(img1,  best_x / 2,  best_y / 2, 0);
  cv::Mat img2_cut = transform(img2, -best_x / 2, -best_y / 2, -best_th);
  cv::Mat canvas3(img1_cut.rows, 2 * img1_cut.cols + 20, CV_8UC1, cv::Scalar(127) );

  img1_cut.copyTo(canvas3(cv::Rect2i(0, 0, img1_cut.cols, img1_cut.rows) ) );
  img2_cut.copyTo(canvas3(cv::Rect2i(img1_cut.cols + 20, 0, img1_cut.cols, img1_cut.rows) ) );

  cv::imshow("img1 (cut) <-> img2 (corrected, cut)", canvas3);
  cv::waitKey(0);
#endif

  cv::Point2f best_rel = camera_matrix.image2relative(
      cv::Point2i(best_x + img1.cols / 2, best_y + img1.rows / 2) )
      + cv::Point2f(pos_prev.x, pos_prev.y);

  return cv::Point3f(best_rel.x, best_rel.y, best_th + pos_prev.z);
}

void Map::update(const cv::Mat &image, const Particle &pos_world,
      const fisheye_camera_matrix::CameraMatrix &_camera_matrix) {
  // update camera_matrix (with respect to auto-calibration, dynamic height, etc.)
  camera_matrix = _camera_matrix;
  /*ready=true;
  return;*/
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
  ros::Time now        = ros::Time::now();
  float dt             = (now - map_piece->stamp).toSec();

  if(path_now != center) {
    path_prev = path_now;
    path_now  = center;
  }

  // update the mappiece if needed
  if(
      !map_piece->is_set
      || dt > update_interval_max
      || (dt > update_interval_min && dist(pos_world.p, center) < dist(map_piece->pos_world, center) )
  ) {
    image.copyTo(map_piece->img);

    map_piece->is_set = true;
    map_piece->stamp  = now;

    // correct the position
    /* TODO Do something smart - full brute force scanning is way too slow!
    if(path_prev != path_now) {
      cv::Point2i path_prev_grid = world2grid(path_prev);

      map_piece->pos_world = image_distance(
          grid.at(path_prev_grid.y).at(path_prev_grid.x).img,
          image, grid.at(path_prev_grid.y).at(path_prev_grid.x).pos_world, pos_world.p);
    }
    else */
      map_piece->pos_world = pos_world.p;
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

inline float Map::dist(const cv::Point3f &p1, const cv::Point3f &p2) {
  const float x = p1.x - p2.x;
  const float y = p1.y - p2.y;

  return sqrtf(x * x + y * y);
}

cv::Mat Map::transform(const cv::Mat &img, const int dx, const int dy, const float rotation) {
  const int cx1   = img.cols / 2;
  const int cy1   = img.rows / 2;
  const int dim_x = std::min(40, img.cols - 2 * abs(dx) );
  const int dim_y = std::min(40, img.rows - 2 * abs(dy) );
  const int cx2   = dim_x / 2;
  const int cy2   = dim_y / 2;
  const float phs = sinf(rotation);
  const float phc = cosf(rotation);

  cv::Mat img_tf(dim_y, dim_x, CV_8UC1);

  for(int c = 0; c < dim_x; ++c) {
    const int sx = c - cx2;

    for(int r = 0; r < dim_y; ++r) {
      const int sy   = r - cy2;
      const float x  = sx + dx;
      const float y  = sy + dy;
      const float xx = x * phc - y * phs + cx1;
      const float yy = x * phs + y * phc + cy1;

      if(xx >= 0 && yy >= 0 && xx < img.cols && yy < img.rows)
        img_tf.at<uchar>(r, c) = img.at<uchar>( rintf(yy), rintf(xx) );
      else
        img_tf.at<uchar>(r, c) = 0;
    }
  }

  return img_tf;
}

}
