#ifndef SRC_PARTICLEFILTER_HPP_
#define SRC_PARTICLEFILTER_HPP_

#include <math.h>
#include <vector>
#include <random>
#include "map.hpp"
#include "image_evaluator.hpp"
#include "particle.hpp"
#include "dbscan.hpp"

namespace cps2 {

struct Bin {
  int x;
  int y;
  float cx;
  float cy;
  int count;
  float belief;
  std::vector<Particle> ps;
};

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
  
  std::vector<Particle> particles;
  Particle best;
  std::random_device rd;  //Will be used to obtain a seed for the random number engine
  std::mt19937 gen; //Standard mersenne_twister_engine
  std::uniform_real_distribution<float> udist_t;

  ParticleFilter(cps2::Map *_map, cps2::ImageEvaluator *_image_evaluator, int _particles_num,
                 float _particles_keep, float _particle_belief_scale,
                 float _particle_stdev_lin, float _particle_stdev_ang,
                 bool _hamid_sampling, float _punishEdgeParticlesRate):
          map(_map),
          image_evaluator(_image_evaluator),
          particles_num(_particles_num),
          particles_keep( (int)(_particles_keep * _particles_num) ),
          particle_belief_scale(_particle_belief_scale * _particle_belief_scale),
          particle_stdev_lin(_particle_stdev_lin),
          particle_stdev_ang(_particle_stdev_ang),
          gen(rd() ),
          udist_t(0, 2 * M_PI),
          best(0, 0, 0),
          hamid_sampling(_hamid_sampling),
          punishEdgeParticlesRate(_punishEdgeParticlesRate)
  {}

  ~ParticleFilter() {}

  void addNewRandomParticles() {
#ifdef DEBUG_PF_STATIC
    particles.push_back(Particle(   0, -2.0,  0*M_PI/4) );
    particles.push_back(Particle( 1.5, -1.5,  1*M_PI/4) );
    particles.push_back(Particle( 2.0,    0,  2*M_PI/4) );
    particles.push_back(Particle( 1.5,  1.5,  3*M_PI/4) );
    particles.push_back(Particle(   0,  2.0,  4*M_PI/4) );
    particles.push_back(Particle(-1.5,  1.5, -3*M_PI/4) );
    particles.push_back(Particle(-2.0,    0, -2*M_PI/4) );
    particles.push_back(Particle(-1.5, -1.5, -1*M_PI/4) );
    return;
#endif

    std::uniform_real_distribution<float> udist_x(
        map->bbox.x, map->bbox.x + map->bbox.width);
    std::uniform_real_distribution<float> udist_y(
        map->bbox.y, map->bbox.y + map->bbox.height);

    for(int i = 0; i < particles_num - particles.size(); ++i) {
      Particle p(udist_x(gen), udist_y(gen), udist_t(gen) );
      particles.push_back(p);
    }
  }

  void motion_update(float dx, float dth) {
    for(std::vector<Particle>::iterator it = particles.begin(); it < particles.end(); ++it) {
      it->p.z += dth;
      it->p.x += dx * cosf(it->p.z);
      it->p.y += dx * sinf(it->p.z);
    }
  }

  void evaluate(cv::Mat &img) {
    best.belief = 0;

    cv::Mat img_tf = image_evaluator->transform(
        img, cv::Point3f(img.cols / 2, img.rows / 2, 0), cv::Size2i(img.cols, img.rows) );

    for(std::vector<Particle>::iterator it = particles.begin(); it != particles.end(); ++it) {
      std::vector<cv::Mat> mappieces = map->get_map_pieces(it->p);

      if(mappieces.empty() )
        it->belief = 0;
      else {

        // TODO handle several mappieces

        float e = image_evaluator->evaluate(img_tf, mappieces.front() );

        it->belief = expf(-particle_belief_scale * (e * e) );

        // punish particles that are outside the map
        if (it->p.x >= (map->bbox.x + map->bbox.width) ||
            it->p.y >= (map->bbox.y + map->bbox.height) ||
            it->p.x < map->bbox.x || it->p.y < map->bbox.y) {
          it->belief *= punishEdgeParticlesRate;
        }

        if(it->belief > best.belief)
          best = *it;
      }
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
               fmax(map->bbox.x, fmin(map->bbox.x + map->bbox.width,  ndist_x(gen) ) ),
               fmax(map->bbox.y, fmin(map->bbox.y + map->bbox.height, ndist_y(gen) ) ),
               ndist_t(gen) );

           new_particles.push_back(new_particle);
         }
     }

    // randomize the remainder
    particles = new_particles;

    addNewRandomParticles();
  }

  Particle getBest(){
    //auto cluster = DBScan().dbscan(particles, 0.001, 1);
    binning();
    return best;
  }

  Particle binning() {
    Bin bins[20][20];
    Bin *bestBin = &(bins[0][0]);

    for(int i = 0; i < 20; ++i)
      for(int j = 0; j < 20; ++j) {
        bins[i][j].x      = j;
        bins[i][j].y      = i;
        bins[i][j].cx     = -5 + j * 0.5;
        bins[i][j].cy     = -5 + i * 0.5;
        bins[i][j].belief = 0;
        bins[i][j].count  = 0;
      }


    for(std::vector<Particle>::iterator it = particles.begin();
        it != particles.end(); ++it) {

      int x = 10 + (int)roundf(2 * it->p.x);
      int y = 10 + (int)roundf(2 * it->p.y);

      bins[y][x].belief += it->belief;
      ++bins[y][x].count;
      bins[y][x].ps.push_back(*it);

      if(bins[y][x].belief > bestBin->belief)
        bestBin = &(bins[y][x]);
    }

    float sx = 0;
    float sy = 0;

    for(std::vector<Particle>::const_iterator it = bestBin->ps.begin();
        it != bestBin->ps.end(); ++it) {

      sx += it->p.x;
      sy += it->p.y;
    }

    sx /= bestBin->count;
    sy /= bestBin->count;

    std::vector<Bin> goodBins;
    goodBins.push_back(*bestBin);

    if(sx < bestBin->cx) {
      if(bestBin->x > 0)
        goodBins.push_back(bins[bestBin->y][bestBin->x - 1]);

      if(sy < bestBin->cy) {
        if(bestBin->y > 0)
          goodBins.push_back(bins[bestBin->y - 1][bestBin->x]);

        if(bestBin->x > 0 && bestBin->y > 0)
          goodBins.push_back(bins[bestBin->y - 1][bestBin->x - 1]);
      }
      else {
        if(bestBin->y < 19)
          goodBins.push_back(bins[bestBin->y + 1][bestBin->x]);

        if(bestBin->x > 0 && bestBin->y < 19)
          goodBins.push_back(bins[bestBin->y + 1][bestBin->x - 1]);
      }
    }
    else {
      if(bestBin->x < 19)
        goodBins.push_back(bins[bestBin->y][bestBin->x + 1]);

      if(sy < bestBin->cy) {
        if(bestBin->y > 0)
          goodBins.push_back(bins[bestBin->y - 1][bestBin->x]);

        if(bestBin->x < 19 && bestBin->y > 0)
          goodBins.push_back(bins[bestBin->y - 1][bestBin->x + 1]);
      }
      else {
        if(bestBin->y < 19)
          goodBins.push_back(bins[bestBin->y + 1][bestBin->x]);

        if(bestBin->x < 19 && bestBin->y < 19)
          goodBins.push_back(bins[bestBin->y + 1][bestBin->x + 1]);
      }
    }

    sx = 0;
    sy = 0;

    float st = 0;
    float sb = 0;

    for(std::vector<Bin>::const_iterator it = goodBins.begin();
        it != goodBins.end(); ++it)
      for(std::vector<Particle>::const_iterator bit = it->ps.begin();
              bit != it->ps.end(); ++bit) {

        float yaw = fmod(bit->p.z, 2 * M_PI);

        if(yaw < 0)
          yaw += 2 * M_PI;

        sx += bit->belief * bit->p.x;
        sy += bit->belief * bit->p.y;
        st += bit->belief * yaw;
        sb += bit->belief;
      }

    Particle bp(sx / sb, sy / sb, st / sb);
    best = bp;
  }
};
} // namespace cps2

#endif
