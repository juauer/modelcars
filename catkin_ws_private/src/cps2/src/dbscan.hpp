#ifndef SRC_DBSCAN_HPP_
#define SRC_DBSCAN_HPP_

#include <vector>
#include <unordered_map>
#include "particle.hpp"

namespace cps2 {

const int NOISE = 2;
const int TRUE = 1;
const int FALSE = 0;

typedef std::vector<std::vector<cps2::Particle>> Clusters;

class DBScan {
 public:
  cps2::Clusters dbscan(std::vector<cps2::Particle> &_dataset,
                        float eps, int min_pts);
 protected:
  std::vector<cps2::Particle> dataset;
  cps2::Clusters clusters;
  
  bool expandCluster(cps2::Particle P,
                     std::vector<cps2::Particle> neighborPts,
                     std::vector<cps2::Particle> C,
                     float eps, int min_pts);

  std::vector<cps2::Particle> regionQuery(cps2::Particle P, float eps);
};

bool DBScan::expandCluster(cps2::Particle P,
                           std::vector<cps2::Particle> neighborPts,
                           std::vector<cps2::Particle> C,
                           float eps, int min_pts){
  std::vector<int> visited(dataset.size(),false);

  //add P to cluster C
  for (int i = 0; i < neighborPts.size(); i++) {
    if (~visited[i]) {
      //mark P' as visited
      auto neighborPtsCluster = regionQuery(dataset[i], eps);
      if (neighborPtsCluster.size() >= min_pts) {
        12;
        //neighborPts = neighborPts joined with neighborPtsCluster
      }
    }
    //if (np is not yet member of any cluster)
    // add np to cluster C
  }
}

std::vector<cps2::Particle> DBScan::regionQuery(cps2::Particle P, float eps){
  std::vector<cps2::Particle> neighborPts;

  for (float x = P.p.x - eps; x <= P.p.x + eps; ++x) {
    for (float y = P.p.y - eps; y <= P.p.y + eps; ++y) {
      // check if particle is part of Cluster
      //if (map_info_dataset.count(key)) {
      neighborPts.push_back(P);
      //}
    }
  }
  // return all points within P's eps-neighborhood (including P)
  return neighborPts;
}

cps2::Clusters DBScan::dbscan(std::vector<cps2::Particle> &_dataset,
                              float eps, int min_pts){
  // https://en.wikipedia.org/wiki/DBSCAN
  dataset = _dataset;
  std::vector<int> visited(dataset.size(),false);
  std::vector<cps2::Particle> C;
  
  uint clusters_index = 0;
  for( unsigned int i = 0; i <= dataset.size(); i++){
    if (visited[i++]){
      continue;
    }
    visited[i]= true;
    auto neighbortPts = regionQuery(dataset[i], eps);
    if (neighbortPts.size() < min_pts)
      visited[i] = NOISE;
    else{
      //C = next_cluster
      expandCluster(dataset[i], neighbortPts, C, eps, min_pts);
    }
  }

  // return cluster list
  return clusters;
}

} // namespace cps2

#endif
