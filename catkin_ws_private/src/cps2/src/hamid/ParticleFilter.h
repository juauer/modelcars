/*
 * ParticleFilter.h
 *
 *  Created on: Dec 9, 2011
 *      Author: hamid
 */

#ifndef PARTICLEFILTER_H_
#define PARTICLEFILTER_H_

#include <vector>

#define NUMIDS 1000

template<class ParticleType>
class ParticleFilter {
 public:

  int particleCount;
  std::vector<ParticleType> particles;
  unsigned int idShitArr[NUMIDS];
  unsigned char shitGrid[100][100];

  ParticleFilter(int size);
  virtual ~ParticleFilter();

  void initializeParticles();
  void evaluateParticles() {
    typename std::vector<ParticleType>::iterator it;
    for (it = particles.begin(); it != particles.end(); it++) {
      it->evaluate();
    }
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
    for (int i = 0; i < nRandom; ++i) {
      particles.push_back(ParticleType());
    }
  }

  void applyShitFactor2(){  //works on the assumption of sorted particles
    memset(idShitArr,0,sizeof(idShitArr));

    for (unsigned int i = 0; i < particles.size(); ++i) {
      int id;
      id = particles[i].ID;
      idShitArr[id] += 1;
    }
    unsigned int idCount[NUMIDS]={0};
    for (unsigned int i = 0; i < particles.size(); ++i) {
      int id;
      id = particles[i].ID;
      idCount[id]++;
      particles[i].belief /= idShitArr[id];
      // if (idCount[id]>10) particles[i].belief=0;
      // idShitArr[id]*=2; //works on the assumption of sorted particles, reduces the belief of further examples
    }
  }

  void applyShitFactor(){
    memset(shitGrid,0,sizeof(shitGrid));

    for (unsigned int i = 0; i < particles.size(); ++i) {
      int px,py;
      particles[i].getShitCoordinates(px,py);
      //			printf("(%d,%d)",px,py);
      shitGrid[px][py] += 1;
    }
    for (unsigned int i = 0; i < particles.size(); ++i) {
      int px,py;
      particles[i].getShitCoordinates(px,py);
      particles[i].belief /= shitGrid[px][py];
    }

    //		IplImage *img = cvCreateImageHeader(cvSize(100,100),IPL_DEPTH_8U,1);
    //		img->imageData = (char*)shitGrid;
    //		cvShowImage("shit",img);
  }

  double sumBeliefs(){
    double sumBel = 0.0;
    for (unsigned int i = 0; i < particles.size(); ++i) {
      sumBel += particles[i].belief;
    }
    return sumBel;
  }

  double bestBelief(){
    double maxBel = 0.0;
    for (unsigned int i = 0; i < particles.size(); ++i) {
      maxBel = maxBel > particles[i].belief ? maxBel : particles[i].belief;
    }
    return maxBel;
  }

  double getBest(){
    //sort()
    //return first item in list    
  }
  
  void sortParticles(){
    std::vector<ParticleType> newParticles;
    typename std::vector<ParticleType>::iterator itOld,itNew;

    for ( itOld = particles.begin( ) ; itOld != particles.end( ) ; itOld++ ){
      for (itNew = newParticles.begin( );
           itNew  != newParticles.end( ) && itNew->belief > itOld->belief; itNew++);
      newParticles.insert(itNew,*itOld);
    }
    particles=newParticles;

  }
  
  void mergeIDs(){
    CvPoint3D32f clusters[NUMIDS];
    unsigned int parent[NUMIDS];
    float widths [NUMIDS];
    //initialize the variables
    for (unsigned int i = 0; i < NUMIDS; ++i) {
      clusters[i].x = 0;
      clusters[i].y = 0;
      clusters[i].z = 0;
      parent[i] = i;
    }
    
    //build averages of the clusters
    for (unsigned int i = 0; i < particles.size(); ++i) {
      clusters[particles[i].ID].x += particles[i].x;
      clusters[particles[i].ID].y += particles[i].y;
      widths[particles[i].ID] += particles[i].width;
      clusters[particles[i].ID].z ++;
    }
    
    for (unsigned int i = 0; i < NUMIDS; ++i) {
      if (clusters[i].z > 0){
        clusters[i].x /= clusters[i].z;
        clusters[i].y /= clusters[i].z;
        widths[i] /= clusters[i].z;
      }
    }
    
    //check for overlapping clusters
    for (unsigned int i = 0; i < NUMIDS; ++i) {
      if (clusters[i].z <= 0) continue;
      for (unsigned int j = i+1; j < NUMIDS; ++j) {
        if(clusters[j].z <= 0) continue;
        float dist = fabs(clusters[i].x-clusters[j].x) + fabs(clusters[i].y - clusters[j].y);
        if (dist < widths[i] * 0.5 && parent[i] == i) parent[i]=j;
      }
    }

    //find the top most parent of the node
    for (unsigned int i = 0; i < NUMIDS; ++i) {
      if (clusters[i].z <= 0) continue;
      unsigned int finalParent = parent[i];
      while (parent[finalParent] != finalParent) finalParent = parent[finalParent];
      parent[i]=finalParent;
    }

    //assign the right ID to each particle
    for (unsigned int i = 0; i < particles.size(); ++i) {
      particles[i].ID = parent[particles[i].ID];
    }
  }

  void splitIDs(){ //finds IDs spread too much and split them
    CvPoint3D32f clusters[NUMIDS];
    unsigned int parent[NUMIDS];
    //initialize the variables
    for (unsigned int i = 0; i < NUMIDS; ++i) {
      clusters[i].x = 0;
      clusters[i].y = 0;
      clusters[i].z = 0;
      parent[i] = i;
    }
    
    //build averages of the clusters
    for (unsigned int i = 0; i < particles.size(); ++i) {
      clusters[particles[i].ID].x += particles[i].x;
      clusters[particles[i].ID].y += particles[i].y;
      clusters[particles[i].ID].z ++;
    }
    
    for (unsigned int i = 0; i < NUMIDS; ++i) {
      if (clusters[i].z > 0){
        clusters[i].x /= clusters[i].z;
        clusters[i].y /= clusters[i].z;
      }
    }
    
    //compare particles to the average if too far, receive a new ID
    for (unsigned int i = 0; i < particles.size(); ++i) {
      float dist = fabs(clusters[particles[i].ID].x - particles[i].x)+
          fabs(clusters[particles[i].ID].y - particles[i].y);
      if (dist > /*20*/ particles[i].width * 0.8) particles[i].ID = (rand() % NUMIDS);
    }
  }
};

template<class ParticleType>
ParticleFilter<ParticleType>::ParticleFilter(int size) {
  particleCount = size;
  for (int i = 0; i < size; i++) {
    ParticleType p;
    p.initializeRandom();
    particles.push_back(p);
  }
}
template<class ParticleType>
ParticleFilter<ParticleType>::~ParticleFilter() {
  particles.clear();
}

template<class ParticleType>
void ParticleFilter<ParticleType>::initializeParticles() {
}

#endif /* PARTICLEFILTER_H_ */
