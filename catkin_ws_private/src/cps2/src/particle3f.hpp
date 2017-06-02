#ifndef SRC_PARTICLE3F_HPP_
#define SRC_PARTICLE3F_HPP_
#define _USE_MATH_DEFINES
 
#include <cmath>
#include <random>
#include <limits.h>
#include "image_evaluator.hpp"
#include "map.hpp"
#include <iostream>
namespace cps2 {

class Particle3f{
 public:
  Particle3f(cps2::Map *_map, float _std_dev):
      map(_map),std_dev(_std_dev),belief(100),ie(*_map, 0){ // cps2::IE_MODE_PIXELS
    initRND();
  }
  ~Particle3f(){};

  void operator=(const Particle3f& _p){
    p = _p.p;
    map = _p.map;
    belief = _p.belief;
  }
  
  void evaluate(cv::Mat &img, float sx, float sy){
    p.x += sx;
    p.y += sy;
    this->belief = ie.evaluate(img, p);
    std::cout << this->belief << " ";
  }

  cps2::Particle3f getNearbyParticle(){
    // do nothing yet
    return *this;
  }

  cv::Point3f p;
  cps2::Map *map;
  cps2::ImageEvaluator ie;
  double belief;
  float std_dev;
 protected:
  void initRND(){
    std::random_device rd;  //Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
    std::uniform_int_distribution<> rand_x(1, map->img_gray.size().width);
    std::uniform_int_distribution<> rand_y(1, map->img_gray.size().height);
    std::uniform_real_distribution<> rand_r(1, 2*M_PI);

    p.x = rand_x(gen);
    p.y = rand_y(gen);
    p.z = rand_r(gen);
  }
};

bool operator>(const Particle3f& lhs, const Particle3f& rhs){
  return lhs.belief > rhs.belief;
}

bool operator<(const Particle3f& lhs, const Particle3f& rhs){
  return lhs.belief < rhs.belief;
}

} // namespace cps2

#endif // SRC_PARTICLE3F_HPP_
