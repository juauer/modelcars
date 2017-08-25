#include <iostream>
#include <vector>

#define DEBUG_DBSCAN

#include "dbscan.hpp"

//g++ -g -std=c++11 -I/opt/ros/kinetic/include/opencv-3.2.0-dev dbscan_main.cpp -o dbscan

int main() {
  std::vector<cps2::Particle> particles;
  particles.push_back(cps2::Particle(0,0,0));
  particles.push_back(cps2::Particle(10,10,0));
  particles.push_back(cps2::Particle(10,10,0));
  particles.push_back(cps2::Particle(10,10,0));
  particles.push_back(cps2::Particle(10,10,0));
  particles.push_back(cps2::Particle(20,20,0));
  particles.push_back(cps2::Particle(20,20,0));
  particles.push_back(cps2::Particle(20,20,0));
  particles.push_back(cps2::Particle(20,20,0));
  particles.push_back(cps2::Particle(20,20,0));
  particles.push_back(cps2::Particle(20,20,0));
  particles.push_back(cps2::Particle(30,30,0));
  particles.push_back(cps2::Particle(40,40,0));
  particles.push_back(cps2::Particle(40,40,0));
  particles.push_back(cps2::Particle(40,40,0));
  particles.push_back(cps2::Particle(40,40,0));
  particles.push_back(cps2::Particle(40,40,0));
  particles.push_back(cps2::Particle(40,40,0));
  particles.push_back(cps2::Particle(50,50,0));
  particles.push_back(cps2::Particle(50,50,0));

  std::cout << "start dbscan ... " << std::endl;
  cps2::DBScan dbscan;
  auto cluster = dbscan.dbscan(particles, 1, 5);

  std::cout << "cluster num: " << cluster.size() << std::endl;
  for (auto c:cluster) {
    std::cout << c.size() << ", ";
  }

}
