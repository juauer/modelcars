#ifndef SRC_DBSCAN_HPP_
#define SRC_DBSCAN_HPP_

#include <vector>
#include <unordered_map>
#include "particle.hpp"

namespace cps2 {

std::vector<std::vector<cps2::Particle>> dbscan3D(std::vector<cps2::Particle> &dataset,
                                                  float eps, int min_pts){
  // https://en.wikipedia.org/wiki/DBSCAN

  std::vector<std::vector<cps2::Particle>> clusters;
  int idx_current_cluster = 0;

  struct info_point {
    cps2::Particle position;
    int cluster;
  };

  std::unordered_map<int, info_point> map_info_dataset;

  auto getKey = [](cps2::Particle P) {
    return ((int64_t)P.p.z << 42) + ((int64_t)P.p.y << 21) + (int64_t)P.p.x;
  };

  for (auto&& P : dataset) {
    map_info_dataset[getKey(P)] = info_point{ P, -1 };
  }

  auto regionQuery = [&eps, &map_info_dataset, &getKey](const int key) {
    const auto& pos_P = map_info_dataset.at(key).position;

    std::vector<int> keys_neighborPts;

    for (int x = pos_P.p.x - eps; x <= pos_P.p.x + eps; ++x) {
      for (int y = pos_P.p.y - eps; y <= pos_P.p.y + eps; ++y) {
        for (int z = pos_P.p.z - eps; z <= pos_P.p.z + eps; ++z) {
          int key = getKey({ x, y, z });

          if (map_info_dataset.count(key)) {
            keys_neighborPts.push_back(key);
          }
        }
      }
    }
    return keys_neighborPts;
  };

  auto expandCluster = [&eps, &min_pts, &map_info_dataset, &clusters, &regionQuery]
      (const int key, std::vector<int> &keys_neighborPts, int idx_current_cluster) {
    clusters.at(idx_current_cluster).push_back(map_info_dataset.at(key).position);
    map_info_dataset.at(key).cluster = idx_current_cluster;

    for (int i = 0; i < keys_neighborPts.size(); ++i) {
      auto &local_key = keys_neighborPts[i];
      auto &local_point = map_info_dataset.at(local_key);

      if (local_point.cluster == -1) {
        auto keys_neighborPts_local_point = regionQuery(local_key);

        if (keys_neighborPts_local_point.size() >= min_pts) {
          keys_neighborPts.insert(keys_neighborPts.end(), keys_neighborPts_local_point.begin(),
                                  keys_neighborPts_local_point.end());
        }

        clusters.at(idx_current_cluster).push_back(local_point.position);
        local_point.cluster = idx_current_cluster;
      }
    }
  };

  for (auto&& P : map_info_dataset) {
    const int& key_P = P.first;
    info_point& info_P = P.second;

    if (info_P.cluster != -1) {
      continue;
    }

    auto keys_neighborPts = regionQuery(key_P);

    if (keys_neighborPts.size() >= min_pts) {
      clusters.emplace_back();
      expandCluster(key_P, keys_neighborPts, idx_current_cluster);
      ++idx_current_cluster;
    }
  }

  return clusters;
}

} // namespace cps2

#endif
