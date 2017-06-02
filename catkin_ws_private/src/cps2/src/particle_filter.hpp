#ifndef SRC_PARTICLEFILTER_HPP_
#define SRC_PARTICLEFILTER_HPP_

#include <vector>
#include <algorithm>
#include <random>
#include <cv.hpp>
#include "map.hpp"
#include "particle3f.hpp"

namespace cps2 {

class ParticleFilter3f {
 public:
  cps2::Map *map;
  int particles_num;
  int particle_keep;
  float particle_stdev;
  std::vector<Particle3f> particles;
  std::random_device rd;  //Will be used to obtain a seed for the random number engine
  std::mt19937 gen; //Standard mersenne_twister_engine
  std::normal_distribution<> rand_p;
  std::normal_distribution<> rand_mw;

  ParticleFilter3f(cps2::Map *_map, int errorfunction, int _particles_num,
                   int _particle_keep, float _particle_stdev):
      map(_map), particles_num(_particles_num), particle_keep(_particle_keep),
      particle_stdev(_particle_stdev) {
    addNewRandomParticles();

    std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
    std::normal_distribution<> rand_p(1, particles_num);
    std::normal_distribution<> rand_mw(1, 1);
   }

  void evaluate(cv::Mat &img, float sx, float sy) {
    for(auto i : particles)
      i.evaluate(img,sx,sy);

    unsigned int index = rand_p(gen);
    float beta = 0.0f;
    auto max = std::max_element (std::begin(particles),std::end(particles));
    float mw = max->belief;
    std::vector<Particle3f> new_particles;
    for(int i = 0; i < particles.size(); i++) {
      beta += rand_mw(gen) * 2.0 * mw;
      while(beta > particles[index].belief){
        beta -= particles[index].belief;
        index = (index +1)%particles_num;
      }
      new_particles.push_back(particles[index]);      
    }
    particles = new_particles;
  }

  double sumBeliefs(){
    double sumBel = 0.0;
    for (unsigned int i = 0; i < particles.size(); ++i) {
      sumBel += particles[i].belief;
    }
    return sumBel;
  }
  
  void resampleParticles() {
    // number of particles to sample
    const int n = particles_num;

     CvRNG rng = cvRNG(-1);
    std::vector<uint32_t> hits(particles.size(), 0);

    double sumBel = sumBeliefs();
    if (sumBel==0) return;

    // stochastic universal sampling
    double sum = 0.0;
    double ptr = cvRandReal(&rng);
    unsigned int i, j = 0;
    for (i = 0; i < particles.size(); i++) {
      sum += particles[i].belief / sumBel * n;
      while (sum > ptr) {
        hits[i]++;
        j++;
        ptr += 1.0;
      }
    }

    // // distribute new particles near good particles
    // std::vector<Particle3f> newParticles;
    // for (unsigned int i = 0; i < hits.size(); ++i) {
    //   // printf ("hit[%d]=%d\r\n", i,hits[i]);
    //   Particle3f &p = particles[i];
    //   for (unsigned int h = 0; h < hits[i]; ++h) {
    //     if (h == 0){// && hits[i]>1) { // left original particle TODO use ID
    //       newParticles.push_back(p);
    //     } else { // resample nearby particles
    //       newParticles.push_back(p.getNearbyParticle());
    //     }
    //   }
    // }
    // particles = newParticles;
    //TODO: use NearbyParticle to choose range for new particles
    addNewRandomParticles();
  }
  
  void addNewRandomParticles(){
    // distribute random particles
    const int nRandom = particles_num - particles.size();
    // erase bad particles
    particles.erase(particles.begin()+(particles_num - nRandom), particles.end());

    // generate new one
    std::vector<Particle3f> gen;
    for(int i=0;i<nRandom;i++){
      gen.push_back(Particle3f(map, particle_stdev));
    }

    // insert into particle vector
    particles.insert(particles.end(),
                     gen.begin(),
                     gen.end());
  }
  
  cps2::Particle3f getBest(){
    // sort
    std::sort(particles.begin(), particles.end(),
              std::greater<cps2::Particle3f>());
    //return first item in list
    return particles.front();
  }
};

} // namespace cps2

#endif
