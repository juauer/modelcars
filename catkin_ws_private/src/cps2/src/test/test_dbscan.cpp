#include <iostream>
#include <vector>
#include <numeric>
#include <string>
#include <functional>

#include "../dbscan.hpp"

//#define DEBUG_DBSCAN
//g++ -g -std=c++11 -I/opt/ros/kinetic/include/opencv-3.2.0-dev test_dbscan.cpp -o dbscan

int main() {
  std::vector<cps2::Particle> particles;
  particles.push_back(cps2::Particle(30,30,0));
  particles.push_back(cps2::Particle(30,30,1));
  particles.push_back(cps2::Particle(30,30,2));
  particles.push_back(cps2::Particle(60,60,3));
  particles.push_back(cps2::Particle(60,60,4));
  particles.push_back(cps2::Particle(60,60,5));
  particles.push_back(cps2::Particle(60,60,6));
  particles.push_back(cps2::Particle(60,60,7));
  particles.push_back(cps2::Particle(60,60,8));
  particles.push_back(cps2::Particle(50,50,9));
  particles.push_back(cps2::Particle(50,50,10));
  particles.push_back(cps2::Particle(50,50,11));
  particles.push_back(cps2::Particle(50,50,12));
  particles.push_back(cps2::Particle(50,50,13));
  particles.push_back(cps2::Particle(40,40,14));
  particles.push_back(cps2::Particle(40,40,15));
  particles.push_back(cps2::Particle(40,40,16));
  particles.push_back(cps2::Particle(40,40,17));
  particles.push_back(cps2::Particle(0,0,18));
  particles.push_back(cps2::Particle(10,10,19));
  particles.push_back(cps2::Particle(20,20,20));
  particles.push_back(cps2::Particle(20,20,21));

  std::cout << "start dbscan ... " << std::endl;
  cps2::DBScan dbscan;
  auto clusters = dbscan.dbscan(particles, 1.0f, 4);

  std::cout << "cluster amount: " << clusters.size() << std::endl;
  std::cout << "cluster [size,avg]: ";
  for (auto c:clusters) {
    std::cout << "[" << c.size() << ",";

    std::cout << std::accumulate(c.begin(), c.end(), 0,
                    [](float sum, const cps2::Particle& curr)
                                 { return sum + curr.p.x; })/c.size();
    std::cout << "], ";
  }
  std::cout << std::endl;

  return 0;
}
