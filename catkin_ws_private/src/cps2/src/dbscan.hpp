#ifndef SRC_DBSCAN_HPP_
#define SRC_DBSCAN_HPP_

#include <vector>
#include <unordered_set>
#include <utility>
#include <iomanip>
#include <math.h>

#include "particle.hpp"

namespace cps2 {

typedef std::vector<cps2::Particle> Point3fList;
typedef std::vector<std::size_t> neighborList;
typedef std::vector<Point3fList> Clusters;

class DBScan {
 public:
  cps2::Clusters dbscan(std::vector<cps2::Particle> &_dataset,
                        float _eps, int _min_pts);
 protected:
  std::vector<cps2::Particle> dataset;
  cps2::Clusters clusters;
  std::unordered_set<int> visited;
  std::unordered_set<int> clustered;
  
  bool expandCluster(std::size_t _p, neighborList _neighbor_pts,
                     std::size_t _C, float _eps, int _min_pts);

  neighborList regionQuery(std::size_t _p, float _eps);
};

cps2::Clusters DBScan::dbscan(std::vector<cps2::Particle> &_dataset,
                              float _eps, int _min_pts){
  // https://en.wikipedia.org/wiki/DBSCAN
  dataset = _dataset;
  std::size_t clusters_index = 0;
  
  for( unsigned int i = 0; i <= dataset.size() -1; ++i){
    // if point is visited skip to the next point
    if (visited.find(i) != std::end(visited)){
      continue;
    }
    // mark point as visited
    visited.insert(i);
    // get neighboring points
    auto neighbor_pts = regionQuery(i, _eps);
    // check if cluster is big enough
    if (neighbor_pts.size() >= _min_pts) {
      // create new cluster
      clusters.emplace_back();
      expandCluster(i, neighbor_pts, clusters_index, _eps, _min_pts);
      clusters_index++;
    } else {
      // put into noise list
      // noise_pts.push_back(dataset.at(i));
    }
  }

  // return cluster list
  return clusters;
}

bool DBScan::expandCluster(std::size_t _p, neighborList _neighbor_pts,
                           std::size_t _C, float _eps, int _min_pts){
  //add p to cluster C
  clusters.at(_C).push_back(dataset.at(_p));
  clustered.insert(_p);

  for (int i = 0; i < _neighbor_pts.size() -1; ++i) {
    if (visited.find(_neighbor_pts.at(i)) == std::end(visited)) {
      //mark p' as visited
      visited.insert(_neighbor_pts.at(i));
      auto exp_neighbor_pts = regionQuery(_neighbor_pts.at(i), _eps);
      // merge point neighbors to the cluster point list
      if (exp_neighbor_pts.size() >= _min_pts) {
        _neighbor_pts.insert(_neighbor_pts.end(),
                             exp_neighbor_pts.begin(),
                             exp_neighbor_pts.end());
      }
      clusters.at(_C).push_back(dataset.at(_neighbor_pts.at(i)));
      clustered.insert(_neighbor_pts.at(i));
      
    } else {
      //if i is not yet member of any cluster add to cluster
      if (clustered.find(_neighbor_pts.at(i)) == std::end(clustered)){
        clusters.at(_C).push_back(dataset.at(_neighbor_pts.at(i)));
      }
    }
  }
}

neighborList DBScan::regionQuery(std::size_t _p, float _eps){
  cps2::Particle p = dataset.at(_p);
  neighborList neighbor_pts = {_p}; // add p to list
  std::size_t i = 0;

  for (auto np : dataset) {

    // check if particle is part of Cluster
    if (visited.find(i) == std::end(visited)) {

      const float dx = np.p.x - p.p.x;
      const float dy = np.p.y - p.p.y;
      float dist = sqrtf(dx * dx + dy * dy);

      if (dist < _eps){
        neighbor_pts.push_back(i);
      }
    }
    ++i;
  }

  //return all points within p's eps-neighborhood (including p)
  return neighbor_pts;
}

} // namespace cps2

#endif
