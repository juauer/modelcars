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
  
  bool expandCluster(cps2::Particle p, std::vector<int> neighbor_pts,
                     unsigned int C, float eps, int min_pts);

  std::vector<int> regionQuery(cps2::Particle p, float eps);
};

bool DBScan::expandCluster(cps2::Particle p, std::vector<int> neighbor_pts,
                           unsigned int C, float eps, int min_pts){
  std::vector<int> visited(dataset.size(),false);

  //add p to cluster C
  for (int i = 0; i < neighbor_pts.size(); i++) {
    if (~visited[i]) {
      //mark p' as visited
      auto neighbor_pts_cur = regionQuery(dataset[i], eps);
      if (neighbor_pts_cur.size() >= min_pts) {
        //neighbor_pts = neighbor_pts joined with neighbor_pts_cur
      }
    }
    //if (np is not yet member of any cluster)
    // add np to cluster C
  }
}

std::vector<int> DBScan::regionQuery(cps2::Particle p, float eps){
  std::vector<int> neighbor_pts;

  for (float x = p.p.x - eps; x <= p.p.x + eps; ++x) {
    for (float y = p.p.y - eps; y <= p.p.y + eps; ++y) {
      // check if particle is part of Cluster
      //if (map_info_dataset.count(key)) {
      // neighbor_pts.push_back(p);
      //}
    }
  }
  // return all points within p's eps-neighborhood (including p)
  return neighbor_pts;
}

cps2::Clusters DBScan::dbscan(std::vector<cps2::Particle> &_dataset,
                              float eps, int min_pts){
  // https://en.wikipedia.org/wiki/DBSCAN
  unsigned int C = 0;
  dataset = _dataset;
  std::vector<int> visited(dataset.size(),false);
  
  uint clusters_index = 0;
  for( unsigned int i = 0; i <= dataset.size(); i++){
    if (visited[i++]){
      continue;
    }
    visited[i]= true;
    auto neighbort_pts = regionQuery(dataset[i], eps);
    if (neighbort_pts.size() < min_pts)
      visited[i] = NOISE;
    else{
      //C = next_cluster
      expandCluster(dataset[i], neighbort_pts, C, eps, min_pts);
    }
  }

  // return cluster list
  return clusters;
}

} // namespace cps2

#endif
