#ifndef SRC_PARTICLEFILTER_HPP_
#define SRC_PARTICLEFILTER_HPP_

#include <vector>
#include <random>
#include "map.hpp"
#include "image_evaluator.hpp"
#include "particle.hpp"

namespace cps2 {

class ParticleFilter {
public:

  /**
   * Initialize a new ParticleFilter.
   *
   * @param _map pointer to a Map
   * @param _image_evaluator pointer to an ImageEvaluator
   * @param _particles_num total number of particles
   * @param _particles_keep percent of particles that are not resampled randomly
   * @param _particle_belief_scale belief=exp( -(pbs * error)^2)
   * @param _particle_stdev_lin standard deviation to generate noise around particles among x,y
   * @param _particle_stdev_ang standard deviation to generate noise for particles rotation
   * @param _hamid_sampling only apply noise to the new, copied particles in terms of SUS
   * @param _bin_size edge length of the bins used for the binning algorithm. Choose 0 to disable.
   * @param _punishEdgeParticlesRate multiplier for belief of particles, which are pushed outside of the map by motion_updates
   * @param _setStartPos true in case of a known start position
   * @param _startPos the start position, if known
   */
  ParticleFilter(cps2::Map *_map, cps2::ImageEvaluator *_image_evaluator, int _particles_num,
                 float _particles_keep, float _particle_belief_scale, float _particle_stdev_lin,
                 float _particle_stdev_ang, bool _hamid_sampling, float _bin_size,
                 float _punishEdgeParticlesRate, bool _setStartPos, cv::Point3f _startPos);

  ~ParticleFilter();

  /**
   * Generate an amount of "particles.size() - particles_num" new Particles, uniformly
   * distributed over the known map area.
   *
   * If setStartPos is set, Particles are generated near startPos.
   */
  void addNewRandomParticles();

  /**
   * Update all Particles with the latest odometry.
   *
   * @param dx distance traveled since last update
   * @param dth current steering
   */
  void motion_update(const float dx, const float dth);

  /**
   * Compute a belief for each Particle using the given ImageEvaluator and newest sensor data.
   *
   * @param img new, undistorted, grayscale image of ceiling cam to evaluate against.
   */
  void evaluate(const cv::Mat &img);

  /**
   * Resample the Particles.
   */
  void resample();

  /**
   * Get the Particle with the highest belief, after calling evaluate(). Make sure not to call
   * resample() in between evaluate() and getBest(), or the evaluation results will be lost.
   *
   * The best Particle is taken from Binning, if turned on. Otherwise the Particle with the
   * highest belief is returned.
   *
   * @return A Particle representing the best guess for the cars position in world frame.
   */
  Particle getBest();

  const int particles_num;
  const int particles_keep;
  const float particle_belief_scale;
  const float particle_stdev_lin;
  const float particle_stdev_ang;
  const float punishEdgeParticlesRate;
  const bool hamid_sampling;
  const bool binning_enabled;
  const float bin_size;
  const cv::Point3f startPos;
  
  std::vector<Particle> particles;

private:
  /**
   * Use a grid to detect clusters of Particles. Find the cluster with the maximum total belief
   * and compute a new Particle from the weighted mean of the Particles from this cluster.
   *
   * Binning tends to avoid cluster-jumping and additionally smooths the result in comparison
   * to the naive approach.
   */
  void binning();

  cps2::Map *map;
  cps2::ImageEvaluator *image_evaluator;

  bool setStartPos;
  Particle best_single;
  Particle best_binning;
  std::random_device rd;
  std::mt19937 gen;
  std::uniform_real_distribution<float> udist_x;
  std::uniform_real_distribution<float> udist_y;
  std::uniform_real_distribution<float> udist_t;
};

} // namespace cps2

#endif
