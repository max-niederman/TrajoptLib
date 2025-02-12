// Copyright (c) TrajoptLib contributors

#include "trajopt/set/RectangularSet2d.h"

#include <cmath>
#include <optional>

#include "trajopt/set/IntervalSet1d.h"
#include "trajopt/solution/SolutionChecking.h"

namespace trajopt {

RectangularSet2d RectangularSet2d::PolarExactSet2d(double r, double theta) {
  return RectangularSet2d{r * std::cos(theta), r * std::sin(theta)};
}

RectangularSet2d RectangularSet2d::R2() {
  return RectangularSet2d{IntervalSet1d::R1(), IntervalSet1d::R1()};
}

std::optional<SolutionError> RectangularSet2d::CheckVector(
    double xComp, double yComp,
    const SolutionTolerances& tolerances) const noexcept {
  auto xCheck = xBound.CheckScalar(xComp, tolerances);
  if (xCheck.has_value()) {
    return SolutionError{fmt::format("x ", xCheck->errorMessage)};
  }
  auto yCheck = yBound.CheckScalar(yComp, tolerances);
  if (yCheck.has_value()) {
    return SolutionError{fmt::format("y ", yCheck->errorMessage)};
  }
  return std::nullopt;
}

bool RectangularSet2d::IsValid() const noexcept {
  return xBound.IsValid() && yBound.IsValid();
}
}  // namespace trajopt
