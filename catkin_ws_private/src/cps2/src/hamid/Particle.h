/*
 * Particle.h
 *
 *  Created on: Dec 9, 2011
 *      Author: hamid
 */

#ifndef PARTICLE_H_
#define PARTICLE_H_

class Particle {
public:
  Particle();
  virtual ~Particle();

  int ID;
  double belief;

  virtual void evaluate(void) = 0;

  virtual void initializeRandom() = 0;

  virtual void initializeAt() = 0;

  virtual void getShitCoordinates(int &px, int &py) = 0;

};

#endif /* PARTICLE_H_ */
