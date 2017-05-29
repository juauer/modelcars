#ifndef SRC_PARTICLE3F_HPP_
#define SRC_PARTICLE3F_HPP_

#include <cv.h>
#include <limits.h>
#include "particle_filter.hpp"
#include "image_evaluator.hpp"
#include "map.hpp"

namespace cps2 {

class Particle3f : public Particle {
 public:
  Particle3f(cv::Mat _map):map(_map){
    CvRNG rng = cvRNG(-1);
    p.x = cvRandReal(&rng) * std::numeric_limits<float>::max();
    p.y = cvRandReal(&rng) * std::numeric_limits<float>::max();
    p.z = cvRandReal(&rng) * CV_2PI;
  }
  virtual ~Particle3f(){};
  virtual void eval(cv::Mat &img, cv::Point3f &particle) {}

  cv::Point3f p;
  cv::Mat map;
};

typedef ParticleFilter<Particle3f> ParticleFilter3f;
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
