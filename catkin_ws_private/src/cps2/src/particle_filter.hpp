#ifndef SRC_PARTICLEFILTER_HPP_
#define SRC_PARTICLEFILTER_HPP_

#include <vector>
#include <random>
#include "map.hpp"
#include "image_evaluator.hpp"
#include "particle.hpp"

namespace cps2 {

class ParticleFilter {
 public:
  cps2::Map *map;
  cps2::ImageEvaluator *image_evaluator;

  const int particles_num;
  const int particles_keep;
  const float particle_belief_scale;
  const float particle_stdev_lin;
  const float particle_stdev_ang;
  const float punishEdgeParticlesRate;
  const bool hamid_sampling;
  const bool binning_enabled;
  const float bin_size;
  bool setStartPos;
  const cv::Point3f startPos;
  
  std::vector<Particle> particles;
  Particle best_single;
  Particle best_binning;
  std::random_device rd;  //Will be used to obtain a seed for the random number engine
  std::mt19937 gen; //Standard mersenne_twister_engine
  std::uniform_real_distribution<float> udist_x;
  std::uniform_real_distribution<float> udist_y;
  std::uniform_real_distribution<float> udist_t;

  ParticleFilter(cps2::Map *_map, cps2::ImageEvaluator *_image_evaluator, int _particles_num,
                 float _particles_keep, float _particle_belief_scale,
                 float _particle_stdev_lin, float _particle_stdev_ang, 
                 bool _hamid_sampling, float _bin_size, float _punishEdgeParticlesRate,
                 bool _setStartPos, cv::Point3f _startPos);

  ~ParticleFilter();

  void addNewRandomParticles();

  void motion_update(const float dx, const float dth);

  void evaluate(const cv::Mat &img);

  void resample();

  Particle getBest();

  void binning();
};

} // namespace cps2

#endif
