#ifndef SRC_PARTICLEFILTER_HPP_
#define SRC_PARTICLEFILTER_HPP_

#include <vector>
#include <algorithm>
#include <cv.h>

namespace cps2 {

template <typename ParticleType>
class ParticleFilter {
 public:
  int particleCount;
  std::vector<ParticleType> particles;

  ParticleFilter(int size);
  void evaluateParticles() {
    std::for_each (particles.begin(),
                   particles.end(),
                   &ParticleType::eval);
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
    std::vector<ParticleType> newParticles;
    for (unsigned int i = 0; i < hits.size(); ++i) {
      // printf ("hit[%d]=%d\r\n", i,hits[i]);
      ParticleType &p = particles[i];
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
    std::vector<ParticleType> inserts(nRandom);
    
    particles.insert(particles.end(),
                     inserts.begin(),
                     inserts.end());
  }
  
  ParticleType getBest(){
    //sort()
    std::sort(particles.begin(), particles.end(),
              std::greater<ParticleType>());
    //return first item in list
    return particles.front();
  }
  
};

template<typename ParticleType>
ParticleFilter<ParticleType>::ParticleFilter(int size) {
  particleCount = size;
  particles = std::vector<ParticleType>(size);
}

class Particle {
 public:
  Particle(){};
  virtual ~Particle(){};

  double belief;

  virtual void eval(cv::Mat &img, cv::Point3f &particle) = 0;
};


} // namespace cps2

#endif
