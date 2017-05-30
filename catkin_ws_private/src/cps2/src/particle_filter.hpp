#ifndef SRC_PARTICLEFILTER_HPP_
#define SRC_PARTICLEFILTER_HPP_

#include <vector>
#include <algorithm>
#include <cv.hpp>
#include "map.hpp"
#include "particle3f.hpp"

namespace cps2 {

class ParticleFilter3f {
 public:
  int particleCount;
  std::vector<Particle3f> particles;
  cps2::Map *map;
  float sigma;

  ParticleFilter3f(cps2::Map *_map, int _particleCount,
                 float _sigma): map(_map), particleCount(_particleCount),
                                sigma(_sigma){
    addNewRandomParticles();
  }

  void eval(cv::Mat &img, float sx, float sy) { 
    for(Particle3f i : particles)
      i.eval(img,sx,sy);
  }

  double sumBeliefs(){
    double sumBel = 0.0;
    for (unsigned int i = 0; i < particles.size(); ++i) {
      sumBel += particles[i].belief;
    }
    return sumBel;
  }
  
  void resampleParticles(int percent) {
    if (percent > 100)
      percent = 100;
    if (percent < 0)
      percent = 0;

    // number of particles to sample
    const int n = percent * particles.size() / 100;

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

    // distribute new particles near good particles
    std::vector<Particle3f> newParticles;
    for (unsigned int i = 0; i < hits.size(); ++i) {
      // printf ("hit[%d]=%d\r\n", i,hits[i]);
      Particle3f &p = particles[i];
      for (unsigned int h = 0; h < hits[i]; ++h) {
        if (h == 0){// && hits[i]>1) { // left original particle TODO use ID
          newParticles.push_back(p);
        } else { // resample nearby particles
          newParticles.push_back(p.getNearbyParticle());
        }
      }
    }
    particles = newParticles;
  }
  
  void addNewRandomParticles(){
    // distribute random particles
    const int nRandom = particleCount - particles.size();
    std::vector<Particle3f> inserts(nRandom);
    
    particles.insert(particles.end(),
                     inserts.begin(),
                     inserts.end());
  }
  
  Particle3f getBest(){
    //sort()
    std::sort(particles.begin(), particles.end(),
              std::greater<Particle3f>());
    //return first item in list
    return particles.front();
  }
};

} // namespace cps2

#endif
