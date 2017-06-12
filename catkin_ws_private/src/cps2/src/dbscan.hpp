#ifndef SRC_DBSCAN_HPP_
#define SRC_DBSCAN_HPP_

#include <vector>
#include <unordered_map>
#include "particle.hpp"

namespace cps2 {

std::vector<std::vector<cps2::Particle>> dbscan3D(std::vector<cps2::Particle> &dataset,
                                                  float eps, int min_pts){
  // https://en.wikipedia.org/wiki/DBSCAN
  std::vector<bool> visited(dataset.size(),false);
  uint clusters_index = 0;
  for( unsigned int i = 0; i <= dataset.size(); i++){
    if (visited[i++]){
      continue;
    }
    visited[i]= true;
  }

  return std::vector<std::vector<cps2::Particle>>();
  //return clusters;
}

} // namespace cps2

#endif
