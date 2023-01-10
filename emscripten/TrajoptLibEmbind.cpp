// Copyright (c) TrajoptLib contributors

#include <drivetrain/SwerveDrivetrain.h>
#include <drivetrain/SwerveModule.h>
#include <emscripten/bind.h>
#include <path/HolonomicPath.h>

#include <OptimalTrajectoryGenerator.h>

using namespace emscripten;
using namespace trajopt;

double add(double a, double b) {
  return a + b;
}

std::string exclaim(std::string message) {
  return message + "!";
}

void opticall() {
  SwerveDrivetrain swerveDrivetrain(45, 6,
                                    {SwerveModule(+0.6, +0.6, 0.04, 70, 2),
                                     SwerveModule(+0.6, -0.6, 0.04, 70, 2),
                                     SwerveModule(-0.6, +0.6, 0.04, 70, 2),
                                     SwerveModule(-0.6, -0.6, 0.04, 70, 2)});
  Obstacle bumpers(0, {{+0.5, +0.5}, {-0.5, +0.5}, {-0.5, -0.5}, {+0.5, -0.5}});
  HolonomicPath holonomicPath(HolonomicPath(
      {HolonomicWaypoint({TranslationConstraint(RectangularSet2d(0, 0)),
                          HeadingConstraint(0)},
                         {VelocityConstraint(RectangularSet2d(0, 0)),
                          AngularVelocityConstraint(0)},
                         {}, {}, 0, {InitialGuessPoint(0, 0, 0.0)}),
       HolonomicWaypoint({TranslationConstraint(RectangularSet2d(4, 0)),
                          HeadingConstraint(0)},
                         {VelocityConstraint(RectangularSet2d(0, 0)),
                          AngularVelocityConstraint(0)},
                         {}, {}, 10, {InitialGuessPoint(4, 0, 0.0)})},
      bumpers));
  // SOLVE
  SwerveSolution solution =
      OptimalTrajectoryGenerator::Generate(swerveDrivetrain, holonomicPath);
}

EMSCRIPTEN_BINDINGS(my_module) {
  function("add", &add);
  function("exclaim", &exclaim);
  function("opticall", &opticall);
}
