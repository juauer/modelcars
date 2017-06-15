#ifndef SRC_PARTICLE_HPP_
#define SRC_PARTICLE_HPP_

#include <opencv2/core.hpp>

namespace cps2 {

class Particle {
public:
  Particle(float x, float y, float th) :
      p(x, y, th), belief(0)
  {}

  ~Particle() {}

  cv::Point3f p;
  float belief;
};

} // namespace cps2
#endif
