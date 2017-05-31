#ifndef SRC_PARTICLE3F_HPP_
#define SRC_PARTICLE3F_HPP_

#include <opencv2/core.hpp>
#include <limits.h>
#include "image_evaluator.hpp"
#include "map.hpp"
#include <iostream>

namespace cps2 {

class Particle3f{
 public:
  Particle3f(cps2::Map *_map, float _std_dev):map(_map),std_dev(_std_dev){
    initRND();
  }
  ~Particle3f(){};

  void operator=(const Particle3f& p){
    this->p = p.p;
    this->map = map;
    this->belief = p.belief;
  }
  
  void evaluate(cv::Mat &img, float sx, float sy){

    // sum = 0.0;
    // for (i in range(len(p)): # calculate mean error
    //     dx = (p[i].x - r.x + (world_size/2.0)) % world_size - (world_size/2.0)
    //     dy = (p[i].y - r.y + (world_size/2.0)) % world_size - (world_size/2.0)
    //     err = sqrt(dx * dx + dy * dy)
    //     sum += err
    // return sum / float(len(p))
  }

  cps2::Particle3f getNearbyParticle(){
    // do nothing yet
    return *this;
  }

  cv::Point3f p;
  cps2::Map *map;
  double belief;
  float std_dev;
 protected:
  void initRND(){
    CvRNG rng = cvRNG(-1);
    p.x = cvRandReal(&rng) * map->img_gray.size().height;
    p.y = cvRandReal(&rng) * map->img_gray.size().width;
    p.z = cvRandReal(&rng) * CV_2PI;
  }
};

bool operator>(const Particle3f& lhs, const Particle3f& rhs){
  return lhs.belief > rhs.belief;
}


// class Particle: ImageEvaluator {
//  public:
//   ParticleFilter(Map &map, int resize_scale,
//                  int kernel_size, float kernel_stddev);
//   ParticleFilter(Map &map);

//   float evaluate(cv::Mat &img, cv::Point3f &particle){
//     ImageEvaluator.evaluate(img,this);
//   }

//  protected:
//   cv::Mat &img;
//   IamgeEvalua;
// };

} // namespace cps2

#endif // SRC_PARTICLE3F_HPP_
