#ifndef SRC_PARTICLEFILTER_HPP_
#define SRC_PARTICLEFILTER_HPP_

#include <math.h>
#include <vector>
#include <random>
#include "map.hpp"
#include "image_evaluator.hpp"

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

class ParticleFilter {
 public:
  cps2::Map *map;
  cps2::ImageEvaluator *evaluator;

  const int particles_num;
  const int particles_keep;
  const float particle_stdev_lin;
  const float particle_stdev_ang;
  const bool hamid_sampling;

  std::vector<Particle> particles;
  Particle best;
  std::random_device rd;  //Will be used to obtain a seed for the random number engine
  std::mt19937 gen; //Standard mersenne_twister_engine

  // distributions for random particles
  std::uniform_real_distribution<float> udist_x;
  std::uniform_real_distribution<float> udist_y;
  std::uniform_real_distribution<float> udist_t;

  ParticleFilter(cps2::Map *_map, int errorfunction, int _particles_num,
      float _particles_keep, float _particle_stdev_lin, float _particle_stdev_ang, bool _hamid_sampling):
          map(_map),
          particles_num(_particles_num),
          particles_keep( (int)(_particles_keep * _particles_num) ),
          particle_stdev_lin(_particle_stdev_lin),
          particle_stdev_ang(_particle_stdev_ang),
          gen(rd() ),
          udist_x(0, _map->img_gray.cols - 1),
          udist_y(0, _map->img_gray.rows - 1),
          udist_t(0, 2 * M_PI),
          best(0, 0, 0),
          hamid_sampling(_hamid_sampling)
  {
    evaluator = new ImageEvaluator(*map, errorfunction);

    addNewRandomParticles();
  }

  ~ParticleFilter() {}

  void addNewRandomParticles() {
    for(int i = 0; i < particles_num - particles.size(); ++i) {
      Particle p(udist_x(gen), udist_y(gen), udist_t(gen) );
      particles.push_back(p);
    }
  }

  void motion_update(float dx, float dy, float dt) {
    for(std::vector<Particle>::iterator it = particles.begin(); it < particles.end(); ++it) {
      it->p.x = fmax(0, fmin(map->img_gray.cols - 1, it->p.x + dx) );
      it->p.y = fmax(0, fmin(map->img_gray.rows - 1, it->p.y + dy) );
      it->p.z = it->p.z + dt;

      if (it->p.x >= (map->img_gray.cols - 1) ||
          it->p.y >= (map->img_gray.rows - 1) ||
          it->p.x <= 0|| it->p.y <= 0){
        it->belief /= 2;
      }
    }
  }

  void evaluate(cv::Mat &img) {
    best.belief = 0;

    for(std::vector<Particle>::iterator it = particles.begin(); it != particles.end(); ++it) {
      it->belief = 1 - evaluator->evaluate(img, it->p);

      if(it->belief > best.belief)
        best = *it;
    }
  }
  
  void resample() {
    // stochastic universal sampling
    std::vector<uint32_t> hits(particles_num, 0);

    float sum_beliefs = 0;

    for(std::vector<Particle>::const_iterator it = particles.begin(); it < particles.end(); ++it)
      sum_beliefs += it->belief;

    if(sum_beliefs == 0.0)
      return;

    float step = sum_beliefs / particles_keep;

    std::uniform_real_distribution<float> rnd(0, step);

    float current = rnd(gen); // random number between 0 and 'step'
    float target  = 0;
    int i         = 0;

    for(std::vector<Particle>::const_iterator it = particles.begin(); it < particles.end(); ++it){
      target += it->belief;

      while(current < target) {
        ++hits[i];
        current += step;
      }

      ++i;
    }

    // distribute new particles near good particles
    std::vector<Particle> new_particles;

    for(int i = 0; i < particles_num; ++i) {
       Particle p = particles[i];

       for(int h = 0; h < hits[i]; ++h)
         if(hamid_sampling && h == 0)
           new_particles.push_back(p);
         else {
           std::normal_distribution<float> ndist_x(p.p.x, particle_stdev_lin * (1 - p.belief) );
           std::normal_distribution<float> ndist_y(p.p.y, particle_stdev_lin * (1 - p.belief) );
           std::normal_distribution<float> ndist_t(p.p.z, particle_stdev_ang * (1 - p.belief) );

           Particle new_particle(
               fmax(0, fmin(map->img_gray.cols - 1, ndist_x(gen) ) ),
               fmax(0, fmin(map->img_gray.rows - 1, ndist_y(gen) ) ),
               ndist_t(gen) );

           new_particles.push_back(new_particle);
         }
     }

    // randomize the remainder
    particles = new_particles;

    addNewRandomParticles();
  }
  
  Particle getBest(){
    return best;
  }
};

} // namespace cps2

#endif
