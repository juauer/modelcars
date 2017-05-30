#ifndef SRC_PARTICLE3F_HPP_
#define SRC_PARTICLE3F_HPP_

#include <cv.h>
#include <limits.h>
#include "image_evaluator.hpp"
#include "map.hpp"
#include <iostream>

namespace cps2 {

class Particle3f{
 public:
  Particle3f(){
    initRND();
  }
  ~Particle3f(){};

  void operator=(const Particle3f& p){
    this->p = p.p;
    this->map = map;
    this->belief = p.belief;
  }
  
  void eval(cv::Mat &img, float sx, float sy){
    std::cout << "eval/n";
    // sum = 0.0;
    // for (i in range(len(p)): # calculate mean error
    //     dx = (p[i].x - r.x + (world_size/2.0)) % world_size - (world_size/2.0)
    //     dy = (p[i].y - r.y + (world_size/2.0)) % world_size - (world_size/2.0)
    //     err = sqrt(dx * dx + dy * dy)
    //     sum += err
    // return sum / float(len(p))
  }

  Particle3f getNearbyParticle(){
    // do nothing yet
    return Particle3f();
  }

  cv::Point3f p;
  cv::Mat map;
  double belief;
 protected:
  void initRND(){
    CvRNG rng = cvRNG(-1);
    p.x = cvRandReal(&rng) * std::numeric_limits<float>::max();
    p.y = cvRandReal(&rng) * std::numeric_limits<float>::max();
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
