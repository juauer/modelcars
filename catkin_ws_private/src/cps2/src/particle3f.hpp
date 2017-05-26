#ifndef SRC_PARTICLE3F_HPP_
#define SRC_PARTICLE3F_HPP_

#include "hamid/Particle.h"
#include "image_evaluator.hpp"
#include "map.hpp"

namespace cps2 {

class Particle3f: public Particle {
 public:
  Particle3f();
  virtual ~Particle3f(){};

  virtual void evaluate(void){}

  virtual void initializeRandom() {};

  virtual void initializeAt() {};

  virtual void getShitCoordinates(int &px, int &py) {};
};

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

