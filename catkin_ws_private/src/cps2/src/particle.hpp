#ifndef SRC_PARTICLE_HPP_
#define SRC_PARTICLE_HPP_

#include <opencv2/core/core.hpp>

namespace cps2 {

struct Particle {
  Particle(const float x, const float y, const float th):p(x, y, th), belief(0){}
  Particle(const cps2::Particle& particle):p(particle.p),belief(particle.belief){}
  ~Particle() {}

  cv::Point3f p;
  float belief;
};

} // namespace cps2
#endif
