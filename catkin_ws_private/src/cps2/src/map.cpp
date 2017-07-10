#include <math.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <ros/package.h>
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
  
  if(!ready)
    return cv::Point3f(0, 0, 0);
  
  cv::Point3f flow_corr;
  cv::Point3f flow_est = pos_now - pos_prev;
  cv::Point2f flow_rel(flow_est.x, flow_est.y);
  cv::Point2i flow_img = camera_matrix.relative2image(flow_rel);
  int x = flow_img.x - img1.cols/2;
  int y = flow_img.y - img1.rows/2;
  float theta = flow_est.z;
  cv::Mat part2 = rotate_cut(img2, -theta, -x, -y);
  cv::Mat part1(img1, cv::Rect(std::max(x, 0), std::max(y, 0), img1.cols-abs(x), img1.rows-abs(y)));
  
  cv::Mat tile1, tile2;
  part1.copyTo(tile1, part2);
  part2.copyTo(tile2, part1);
  cv::imshow("b1",tile1);
  cv::imshow("b2",tile2);
  cv::waitKey(0);
  
  float grad1 = gradient(tile1);
  float grad2 = gradient(tile2);
  float d_theta = grad1-grad2;
    printf("--%f      %f     %f\n",grad1,grad2,d_theta);
  
  flow_corr.z = theta + d_theta;
    
  cv::Mat img2_rot = rotate_img(img2, -flow_corr.z);
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
        flow_img.x = x+dx + img1.cols/2;
        flow_img.y = y+dy + img1.rows/2;
      }
    }
  }
  
  flow_rel = camera_matrix.image2relative(flow_img);
  flow_corr.x = cosf(pos_prev.z)*flow_rel.x - sinf(pos_prev.z)*flow_rel.y;
  flow_corr.y = sinf(pos_prev.z)*flow_rel.x + cosf(pos_prev.z)*flow_rel.y;
  
  return flow_corr;
}

std::vector<cv::Mat> Map::get_map_pieces(const cv::Point3f &pos_world) {
  std::vector<cv::Mat> map_piece_images;

  if(!ready)
    return map_piece_images;

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
  if(!map_pieces[0].is_set)
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
    image.copyTo(map_piece->img);

    map_piece->is_set = true;
    map_piece->stamp  = now;

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

cv::Mat Map::rotate_cut(const cv::Mat &img, const float radiant, const int dx, const int dy) {
  const float r_sin = sinf(radiant);
  const float r_cos = cosf(radiant);
  
  cv::Mat img_rot(img.rows-abs(dy), img.cols-abs(dx), CV_8UC1);
  int delta_x = 0;
  int delta_y = 0;
  if(dx > 0) delta_x = dx;
  if(dy > 0) delta_y = dy;
  
  for(int c = 0; c < img.cols-abs(dx); ++c) {
    const int sx = c - img.cols/2;
    
    for(int r = 0; r < img.rows-abs(dy); ++r) {
      const int sy = r - img.rows/2;
      const float x = sx + delta_x + img.cols/2; // + cosf(radiant)*(x+delta_x-img.cols/2) - sinf(radiant)*(y+delta_y-img.rows/2);
      const float y = sy + delta_y + img.rows/2; // + sinf(radiant)*(x+delta_x-img.cols/2) + cosf(radiant)*(y+delta_y-img.rows/2);
      const float xx = (x - img.cols / 2) * r_cos - (y - img.rows / 2) * r_sin + img.cols/2;
      const float yy = (x - img.cols / 2) * r_sin + (y - img.rows / 2) * r_cos + img.rows/2;
      
      if(xx >= 0 && yy >= 0 && xx < img.cols && yy < img.rows)
        img_rot.at<uchar>(r, c) = img.at<uchar>(yy, xx);
      else
        img_rot.at<uchar>(r, c) = 0;
    }
  }
  return img_rot;
}

/*    cv::Mat ImageEvaluator::transform(const cv::Mat &img,
    const float ph)
{
  const int cx1   = img.cols / 2;
  const int cy1   = img.rows / 2;
  const int dim_x = img.cols;
  const int dim_y = img.rows;
  const int cx2   = dim_x / 2;
  const int cy2   = dim_y / 2;
  const float phs = sinf(ph);
  const float phc = cosf(ph);

  cv::Mat img_tf(dim_y, dim_x, CV_8UC1);

  for(int c = 0; c < dim_x; ++c) {
    const int sx = c - cx2;

    for(int r = 0; r < dim_y; ++r) {
      const int sy   = r - cy2;
      const float x  = (sx);
      const float y  = (sy);
      const float xx = (x - cx1) * phc - (y - cy1) * phs + cx1;
      const float yy = (x - cx1) * phs + (y - cy1) * phc + cy1;

      if(xx >= 0 && yy >= 0 && xx < img.cols && yy < img.rows)
        img_tf.at<uchar>(r, c) = applyKernel(img, (int)xx, (int)yy);
      else
        img_tf.at<uchar>(r, c) = 0;
    }
  }

  return img_tf;
}
  */  
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
