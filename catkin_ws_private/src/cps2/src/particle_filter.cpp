#include <math.h>
#include <stdlib.h>
#include "dbscan.hpp"

#include "particle_filter.hpp"

namespace cps2 {

struct Bin {
  int x;
  int y;
  float cx;
  float cy;
  int count;
  float belief;
  std::vector<Particle> ps;
};

ParticleFilter::ParticleFilter(cps2::Map *_map, cps2::ImageEvaluator *_image_evaluator,
    int _particles_num, float _particles_keep, float _particle_belief_scale,
    float _particle_stdev_lin, float _particle_stdev_ang, bool _hamid_sampling,
    float _bin_size, float _punishEdgeParticlesRate, bool _setStartPos, cv::Point3f _startPos):
        map(_map),
        image_evaluator(_image_evaluator),
        particles_num(_particles_num),
        particles_keep( (int)(_particles_keep * _particles_num) ),
        particle_belief_scale(_particle_belief_scale * _particle_belief_scale),
        particle_stdev_lin(_particle_stdev_lin),
        particle_stdev_ang(_particle_stdev_ang),
        gen(rd() ),
        udist_t(0, 2 * M_PI),
        best_single(0, 0, 0),
        best_binning(0, 0, 0),
        hamid_sampling(_hamid_sampling),
        binning_enabled(_bin_size > 0),
        bin_size(_bin_size > 0 ? _bin_size : 0),
        punishEdgeParticlesRate(_punishEdgeParticlesRate),
        setStartPos(_setStartPos), startPos(_startPos)
{}

ParticleFilter::~ParticleFilter() {}

void ParticleFilter::addNewRandomParticles() {
#ifdef DEBUG_PF_STATIC
  // generate fixed Particles on a circle around startPos
  particles.clear();
  particles.push_back(Particle(startPos.x +   0, startPos.y - 1.0,  0 * M_PI/4) );
  particles.push_back(Particle(startPos.x + 0.7, startPos.y - 0.7,  1 * M_PI/4) );
  particles.push_back(Particle(startPos.x + 1.0, startPos.y +   0,  2 * M_PI/4) );
  particles.push_back(Particle(startPos.x + 0.7, startPos.y + 0.7,  3 * M_PI/4) );
  particles.push_back(Particle(startPos.x +   0, startPos.y + 1.0,  4 * M_PI/4) );
  particles.push_back(Particle(startPos.x - 0.7, startPos.y + 0.7, -3 * M_PI/4) );
  particles.push_back(Particle(startPos.x - 1.0, startPos.y +   0, -2 * M_PI/4) );
  particles.push_back(Particle(startPos.x - 0.7, startPos.y - 0.7, -1 * M_PI/4) );
  return;
#endif

  // fill particles with new, uniformly distributed particles.
  if(setStartPos) {
    // only in first frame: generate near startPos
    const float psdl2 = particle_stdev_lin / 2;

    udist_x.param(std::uniform_real_distribution<float>::param_type(
        startPos.x - psdl2, startPos.x + psdl2) );
    udist_y.param(std::uniform_real_distribution<float>::param_type(
        startPos.y - psdl2, startPos.y + psdl2) );

    setStartPos = false;
  } else {
    // generate all over the map
    udist_x.param(std::uniform_real_distribution<float>::param_type(
        map->bbox.x, map->bbox.x + map->bbox.width) );
    udist_y.param(std::uniform_real_distribution<float>::param_type(
        map->bbox.y, map->bbox.y + map->bbox.height) );
    }

  for(int i = 0; i < particles_num - particles.size(); ++i) {
    const Particle p(udist_x(gen), udist_y(gen), udist_t(gen) );

    particles.push_back(p);
  }
}

void ParticleFilter::motion_update(const float dx, const float dth) {
  for(std::vector<Particle>::iterator it = particles.begin(); it < particles.end(); ++it) {
    it->p.z += dth;
    it->p.x += dx * cosf(it->p.z);
    it->p.y += dx * sinf(it->p.z);
  }
}

void ParticleFilter::evaluate(const cv::Mat &img) {
  best_single.belief = 0;

  // set up img by applying the same blur and downscale which were applied to the mappieces
  const cv::Mat img_tf = image_evaluator->transform(
      img, cv::Point2i(img.cols / 2, img.rows / 2), 0, 0);

  // evaluate all particles against img
  for(std::vector<Particle>::iterator it = particles.begin(); it != particles.end(); ++it) {
    // get a list of mappieces near this particle
    std::vector<cv::Mat> mappieces = map->get_map_pieces(it->p);
    it->belief = 0;

    if(mappieces.empty() )
      continue;
    else {
      // do the actual evaluation. Sum up the beliefs to compute a mean
      for(std::vector<cv::Mat>::iterator jt = mappieces.begin(); jt != mappieces.end(); ++jt) {
        const float e = image_evaluator->evaluate(img_tf, *jt);

        it->belief += expf(-particle_belief_scale * e * e);
      }

      it->belief /= mappieces.size();

      // punish particles that are outside the map
      if (it->p.x >= (map->bbox.x + map->bbox.width)
          || it->p.y >= (map->bbox.y + map->bbox.height)
          || it->p.x < map->bbox.x || it->p.y < map->bbox.y
      )
        it->belief *= punishEdgeParticlesRate;

      // track the best particle
      if(it->belief > best_single.belief)
        best_single = *it;
    }
  }

  // if binning is enabled, run it now
  if(binning_enabled && best_single.belief != 0)
    binning();
}

void ParticleFilter::resample() {
  // stochastic universal sampling
  std::vector<uint32_t> hits(particles_num, 0);

  // sum up the beliefs of all Particles
  float sum_beliefs = 0;

  for(std::vector<Particle>::const_iterator it = particles.begin(); it < particles.end(); ++it)
    sum_beliefs += it->belief;

  if(sum_beliefs == 0.0)
    return;

  // calculate the size of 'one unit' and generate a random start value in that range
  const float step = sum_beliefs / particles_keep;

  std::uniform_real_distribution<float> rnd(0, step);

  float current = rnd(gen);
  float target  = 0;
  int i         = 0;

  // iterate over the Particles and count how often they got 'hit' by SUS. Particles with a
  // higher belief will be hit more often.
  for(std::vector<Particle>::const_iterator it = particles.begin();
      it < particles.end(); ++it)
  {
    target += it->belief;

    while(current < target) {
      ++hits[i];
      current += step;
    }

    ++i;
  }

  // distribute new particles near good particles
  std::vector<Particle> new_particles;

  for(int i = 0; i < particles_num; ++i) {
    const Particle p = particles[i];

    // generate a particle for each hit
    for(int h = 0; h < hits[i]; ++h)
      if(hamid_sampling && h == 0)
        new_particles.push_back(p);
      else {
        // apply some noise. The amount of noise is scaled by the belief of a Particle
        std::normal_distribution<float> ndist_x(p.p.x, particle_stdev_lin * (1 - p.belief) );
        std::normal_distribution<float> ndist_y(p.p.y, particle_stdev_lin * (1 - p.belief) );
        std::normal_distribution<float> ndist_t(p.p.z, particle_stdev_ang * (1 - p.belief) );

        const Particle new_particle(
            fmax(map->bbox.x, fmin(map->bbox.x + map->bbox.width,  ndist_x(gen) ) ),
            fmax(map->bbox.y, fmin(map->bbox.y + map->bbox.height, ndist_y(gen) ) ),
            ndist_t(gen) );

        new_particles.push_back(new_particle);
      }
  }

  // randomize the remainder
  particles = new_particles;

  addNewRandomParticles();
}

Particle ParticleFilter::getBest(){
  // auto cluster = DBScan().dbscan(particles, 0.001, 1);

  if(binning_enabled)
    return best_binning;

  return best_single;
}

void ParticleFilter::binning() {
  // generate a grid covering all of the current map
  const int num_x = (int)ceilf(map->bbox.width  / bin_size);
  const int num_y = (int)ceilf(map->bbox.height / bin_size);

  Bin bins[num_y][num_x];
  Bin *bestBin = &(bins[0][0]);

  // set the grid indices and world frame coords of grid cell center for each Bin
  for(int i = 0; i < num_y; ++i)
    for(int j = 0; j < num_x; ++j) {
      bins[i][j].x      = j;
      bins[i][j].y      = i;
      bins[i][j].cx     = map->bbox.x + (j + 0.5) * bin_size;
      bins[i][j].cy     = map->bbox.y + (i + 0.5) * bin_size;
      bins[i][j].belief = 0;
      bins[i][j].count  = 0;
    }

  // assign the nearest Bin to each Particle and sum up the beliefs for the Bins. Keep track
  // of the Bin with the highest belief
  for(std::vector<Particle>::const_iterator it = particles.begin();
      it != particles.end(); ++it)
  {
    const int x = std::max(0, std::min(num_x - 1,
        (int)floorf( (it->p.x - map->bbox.x) / bin_size) ) );
    const int y = std::max(0, std::min(num_y - 1,
        (int)floorf( (it->p.y - map->bbox.y) / bin_size) ) );

    bins[y][x].belief += it->belief;
    ++bins[y][x].count;

    bins[y][x].ps.push_back(*it);

    if(bins[y][x].belief > bestBin->belief)
      bestBin = &(bins[y][x]);
  }

  // compute the mean (x,y)-position of Particles in the best Bin
  float sx = 0;
  float sy = 0;

  for(std::vector<Particle>::const_iterator it = bestBin->ps.begin();
      it != bestBin->ps.end(); ++it)
  {
    sx += it->p.x;
    sy += it->p.y;
  }

  sx /= bestBin->count;
  sy /= bestBin->count;

  // in order to get a representing cluster that is not compromised by the grid-discretization,
  // add the three Bins nearest to bestBin (and bestBin) to the cluster.
  std::vector<Bin> goodBins;
  goodBins.push_back(*bestBin);

  if(sx < bestBin->cx) {
    if(bestBin->x > 0)
      goodBins.push_back(bins[bestBin->y][bestBin->x - 1]);

    if(sy < bestBin->cy) {
      if(bestBin->y > 0)
        goodBins.push_back(bins[bestBin->y - 1][bestBin->x]);

      if(bestBin->x > 0 && bestBin->y > 0)
        goodBins.push_back(bins[bestBin->y - 1][bestBin->x - 1]);
    }
    else {
      if(bestBin->y < num_y - 1)
        goodBins.push_back(bins[bestBin->y + 1][bestBin->x]);

      if(bestBin->x > 0 && bestBin->y < num_y - 1)
        goodBins.push_back(bins[bestBin->y + 1][bestBin->x - 1]);
    }
  }
  else {
    if(bestBin->x < num_x - 1)
      goodBins.push_back(bins[bestBin->y][bestBin->x + 1]);

    if(sy < bestBin->cy) {
      if(bestBin->y > 0)
        goodBins.push_back(bins[bestBin->y - 1][bestBin->x]);

      if(bestBin->x < num_x - 1 && bestBin->y > 0)
        goodBins.push_back(bins[bestBin->y - 1][bestBin->x + 1]);
    }
    else {
      if(bestBin->y < num_y - 1)
        goodBins.push_back(bins[bestBin->y + 1][bestBin->x]);

      if(bestBin->x < num_x - 1 && bestBin->y < num_y - 1)
        goodBins.push_back(bins[bestBin->y + 1][bestBin->x + 1]);
    }
  }

  // compute the weighted mean of positions for all Particles in the cluster. To compute
  // a mean of angles, split the angles in sin and cos portions, take the weighted means
  // of both and rebuild an angle using atan2.
  sx = 0;
  sy = 0;

  float sts = 0;
  float stc = 0;
  float sb  = 0;

  for(std::vector<Bin>::const_iterator it = goodBins.begin();
      it != goodBins.end(); ++it)
    for(std::vector<Particle>::const_iterator bit = it->ps.begin();
            bit != it->ps.end(); ++bit)
    {
      sx  += bit->belief * bit->p.x;
      sy  += bit->belief * bit->p.y;
      sts += bit->belief * sinf(bit->p.z);
      stc += bit->belief * cosf(bit->p.z);
      sb  += bit->belief;
    }

  const Particle bp(sx / sb, sy / sb, atan2f(sts / sb, stc / sb) );
  best_binning = bp;
}
} // namespace cps2
