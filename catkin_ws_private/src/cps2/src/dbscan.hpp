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
typedef std::vector<cps2::Particle> Point3fList;

class DBScan {
 public:
  cps2::Clusters dbscan(std::vector<cps2::Particle> &_dataset,
                        float _eps, int _min_pts);
 protected:
  std::vector<cps2::Particle> dataset;
  cps2::Clusters clusters;
  std::vector<int> visited;
  
  bool expandCluster(cps2::Particle _p, Point3fList _neighbor_pts,
                     Point3fList _C, float _eps, int _min_pts);

  Point3fList regionQuery(cps2::Particle _p, float _eps);
};

cps2::Clusters DBScan::dbscan(std::vector<cps2::Particle> &_dataset,
                              float _eps, int _min_pts){
  // https://en.wikipedia.org/wiki/DBSCAN
  Point3fList C;
  dataset = _dataset;
  visited.resize(dataset.size(),false);
  
  uint clusters_index = 0;
  for( unsigned int i = 0; i <= dataset.size() -1; i++){
#ifdef DEBUG_DBSCAN
    std::cout << "loop i: " << i << " p.x: " << dataset.at(i).p.x << std::endl;
#endif
    // if point is visited skip to the next point
    if (visited[i]){
#ifdef DEBUG_DBSCAN
      std::cout << "skip: " << i << "\n";
#endif
      continue;
    }
    // mark point as visited
    visited[i]= true;
    // get neighboring points
    auto neighbort_pts = regionQuery(dataset.at(i), _eps);
    // check if cluster is big enough
    if (neighbort_pts.size() > _min_pts) {
      // create new cluster
      clusters.push_back(C);
      C.clear();
      expandCluster(dataset.at(i), neighbort_pts, C, _eps, _min_pts);
    }
  }

  // return cluster list
  return clusters;
}

bool DBScan::expandCluster(cps2::Particle _p, Point3fList _neighbor_pts,
                           Point3fList _C, float _eps, int _min_pts){
  //add p to cluster C
  _C.push_back(_p);

  for (int i = 0; i < _neighbor_pts.size() -1; i++) {
    if (~visited[i]) {
      //mark p' as visited
      visited[i] = true;
      auto exp_neighbor_pts = regionQuery(dataset.at(i), _eps);
      // merge point neighbors to the cluster point list
      if (exp_neighbor_pts.size() >= _min_pts) {
        // _neighbor_pts.insert(_neighbor_pts.end()-1,
        //                      exp_neighbor_pts.begin(),
        //                      exp_neighbor_pts.end());
      }
    } else {
      //if (np is not yet member of any cluster)
      // add np to cluster C
      _C.push_back(_p);
    }
  }
}

Point3fList DBScan::regionQuery(cps2::Particle _p, float _eps){
  Point3fList neighbor_pts;
  neighbor_pts.push_back(_p);
  
  for (auto np : dataset) {
    cps2::Particle diff(np.p.x -_p.p.x, np.p.y -_p.p.y, 0);
    float dist = cv::sqrt(diff.p.x * diff.p.x + diff.p.y * diff.p.y);
    if (dist < _eps){
      // check if particle is part of Cluster
      //if (map_info_dataset.count(key)) {
      neighbor_pts.push_back(np);
    }
  }
  //return all points within p's eps-neighborhood (including p)
  return neighbor_pts;
}

} // namespace cps2

#endif
