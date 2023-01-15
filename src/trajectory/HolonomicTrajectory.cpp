// Copyright (c) TrajoptLib contributors

#include "trajectory/HolonomicTrajectory.h"

#include <memory>

#include "solution/HolonomicSolution.h"

namespace trajopt {

// HolonomicTrajectory::HolonomicTrajectory(
//     std::vector<HolonomicTrajectorySample> samples)
//     : samples(std::move(samples)) {}

HolonomicTrajectory HolonomicTrajectory::FromSolution(const HolonomicSolution& solution) {
  std::vector<HolonomicTrajectorySample> samples;
  double ts = 0.0;
  for (size_t samp = 0; samp < solution.x.size(); samp++) {
    if (samp != 0) {
      ts += solution.dt[samp - 1];
    }
    samples.emplace_back(HolonomicTrajectorySample{
                         ts, solution.x[samp], solution.y[samp],
                         solution.theta[samp], solution.vx[samp],
                         solution.vy[samp], solution.omega[samp]});
  }
  return {samples};
}
}  // namespace trajopt
