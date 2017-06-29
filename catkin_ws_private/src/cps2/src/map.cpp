#include <math.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "map.hpp"

namespace cps2 {

Map::Map(cps2::ImageEvaluator *_image_evaluator, float _grid_size,
    float _update_interval_min, float _update_interval_max)
    : grid_size(_grid_size),
      update_interval_min(_update_interval_min),
      update_interval_max(_update_interval_max),
      ready(false),
      bbox(0, 0, _grid_size, _grid_size),
      image_evaluator(_image_evaluator),
      path_now(cv::Point3f(0, 0, 0) ),
      path_prev(cv::Point3f(0, 0, 0) )
{
  // start with a 1x1 grid
  std::vector<MapPiece> v;
  v.push_back(MapPiece() );
  grid.push_back(v);
}

Map::~Map() {

}

cv::Point3f Map::image_distance(const cv::Mat &img1, const cv::Mat &img2, const cv::Point3f &pos_prev, const cv::Point3f &pos_now) {
  // differenz zwischen Bildern in Weltkoordinaten
  
  // initializing CameraMatrix
  if(camera_matrix.width == 0) {
    fisheye_camera_matrix::CameraMatrix cm(640, 480, 0, 0, 320, 2.55, 1.5);
    camera_matrix = cm;
  }
  
  cv::Point3f flow_corr;
  cv::Point3f flow_est = pos_now - pos_prev;

  cv::Point2f flow_rel(flow_est.x, flow_est.y);
  cv::Point2i flow_img = camera_matrix.relative2image(flow_rel);
  int x = flow_img.x;
  int y = flow_img.y;
  float theta = flow_est.z;
  
  cv::Mat img2_rot = rotate_img(img2, theta);
  
  cv::Mat part1(img1, cv::Rect(std::max(x, 0), std::max(y, 0), img1.cols-abs(x), img1.rows-abs(y)));
  cv::Mat part2(img2_rot, cv::Rect(std::max(-x, 0), std::max(-y, 0), img1.cols-abs(x), img1.rows-abs(y)));

  cv::Mat tile1, tile2;
  part1.copyTo(tile1, part2);
  part2.copyTo(tile2, part1);
  
  float grad1 = gradient(tile1);
  float grad2 = gradient(tile2);
  float d_theta = grad1-grad2;
  
  flow_corr.z = theta + d_theta;
    
  img2_rot = rotate_img(img2, flow_corr.z);
  
  float min_error = 1000;
  
  for (int dx=-10; dx < 11; ++dx)
  {
    for (int dy=-10; dy < 11; ++dy)
    {
      cv::Mat parta(img1, cv::Rect(std::max(x+dx, 0), std::max(y+dy, 0), img1.cols-abs(x+dx), img1.rows-abs(y+dy)));
      cv::Mat partb(img2_rot, cv::Rect(std::max(-x-dx, 0), std::max(-y-dy, 0), img1.cols-abs(x+dx), img1.rows-abs(y+dy)));

      parta.copyTo(tile1, partb);
      partb.copyTo(tile2, parta);

      cv::Mat t = tile1 - tile2;
      if (min_error > cv::sum(cv::mean(t))[0])
      {
        min_error = cv::sum(cv::mean(t))[0];
        flow_img.x = x+dx;
        flow_img.y = y+dy;
      }
    }
  }
  flow_rel = camera_matrix.image2relative(flow_img);
  flow_corr.x = cosf(pos_prev.z)*flow_rel.x - sinf(pos_prev.z)*flow_rel.y;
  flow_corr.y = sinf(pos_prev.z)*flow_rel.x + cosf(pos_prev.z)*flow_rel.y;
  
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

 // map_pieces.push_back(image_evaluator->transform(
  //    theOnePiece.img, pos_image3f, cv::Size2i(camera_matrix.width, camera_matrix.height) ) );

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
    map_piece->img    = image;
    map_piece->is_set = true;

    // TODO correct the position. Maybe like this:
    /*if(path_prev != path_now) {
      cv::Point2i path_prev_grid = world2grid(path_prev);

      map_piece->pos_world = pos_world.p + image_distance(
          grid.at(path_prev_grid.y).at(path_prev_grid.x).img,
          image, pos_world.p - grid.at(path_prev_grid.y).at(path_prev_grid.x).pos_world);
	
	// gives back vector in world coordinates depending on prev position
	  map_piece->pos_world = grid.at(path_prev_grid.y).at(path_prev_grid.x).pos_world + image_distance(
          grid.at(path_prev_grid.y).at(path_prev_grid.x).img,
          image, grid.at(path_prev_grid.y).at(path_prev_grid.x).pos_world, pos_world.p );
    }
    else*/
      map_piece->pos_world = pos_world.p;
      map_piece->stamp     = now;
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

cv::Mat Map::rotate_img(const cv::Mat &img, const float radiant) {
  cv::Mat img_rot(img.rows, img.cols, CV_8UC1);
  for(int x = 0; x < img.cols; ++x) {
    for(int y = 0; y < img.rows; ++y) {
      float x_rot = img.cols/2 + cosf(radiant)*(x-img.cols/2) - sinf(radiant)*(y-img.rows/2);
      float y_rot = img.rows/2 + sinf(radiant)*(x-img.cols/2) + cosf(radiant)*(y-img.rows/2);
      
      if(x_rot >= 0 && y_rot >= 0 && x_rot < img.cols && y_rot < img.rows)
        img_rot.at<uchar>(y, x) = img.at<uchar>(y_rot, x_rot);
      else
        img_rot.at<uchar>(y, x) = 0;
    }
  }
  return img_rot;
}

float Map::gradient(const cv::Mat &img) {
  cv::Mat mat_x, mat_y, abs_x, abs_y;
  cv::Sobel(img,mat_x, CV_16S, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT);
  cv::Sobel(img,mat_y, CV_16S, 0, 1, 3, 1, 0, cv::BORDER_DEFAULT);
  cv::convertScaleAbs(mat_x, abs_x);
  cv::convertScaleAbs(mat_y, abs_y);
  cv::Mat orientation = cv::Mat::zeros(abs_x.rows, abs_y.cols, CV_32F);
  abs_x.convertTo(mat_x,CV_32F);
  abs_y.convertTo(mat_y,CV_32F);
  cv::phase(mat_x, mat_y, orientation, false);
  return cv::mean(orientation)[0];
}

}
