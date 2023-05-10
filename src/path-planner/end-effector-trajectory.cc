// Copyright (c) 2019 CNRS
// Authors: Joseph Mirabel
//

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.

#include <iostream>
#include <typeinfo>
using namespace std;
#include <hpp/constraints/differentiable-function.hh>
#include <hpp/constraints/implicit.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/core/config-validations.hh>
#include <hpp/core/configuration-shooter.hh>
#include <hpp/core/edge.hh>
#include <hpp/core/nearest-neighbor.hh>
#include <hpp/core/node.hh>
#include <hpp/core/path-planner.hh>
#include <hpp/core/path-planning-failed.hh>
#include <hpp/core/path-projector.hh>
#include <hpp/core/path-projector/recursive-hermite.hh>
#include <hpp/core/path-validation.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/core/path.hh>
#include <hpp/core/path/hermite.hh>
#include <hpp/core/problem-target/goal-configurations.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/roadmap.hh>
#include <hpp/core/steering-method.hh>
#include <hpp/manipulation/path-planner/end-effector-trajectory.hh>
#include <hpp/manipulation/steering-method/end-effector-trajectory.hh>
#include <hpp/pinocchio/device-sync.hh>
#include <hpp/pinocchio/liegroup-element.hh>
#include <hpp/pinocchio/util.hh>
#include <hpp/util/debug.hh>
#include <hpp/util/exception-factory.hh>
#include <hpp/util/timer.hh>
#include <pinocchio/multibody/data.hpp>
#include <tuple>

namespace hpp {
namespace manipulation {
namespace pathPlanner {
typedef manipulation::steeringMethod::EET_PIECEWISE SM_t;
typedef manipulation::steeringMethod::EET_PIECEWISEPtr_t SMPtr_t;
unsigned long int uint_infty =
    std::numeric_limits<unsigned long int>::infinity();

EET_PIECEWISEPtr_t EET_PIECEWISE::create(
    const core::ProblemConstPtr_t& problem) {
  EET_PIECEWISEPtr_t ptr(new EET_PIECEWISE(problem));
  ptr->init(ptr);
  return ptr;
}

EET_PIECEWISEPtr_t EET_PIECEWISE::createWithRoadmap(
    const core::ProblemConstPtr_t& problem, const core::RoadmapPtr_t& roadmap) {
  EET_PIECEWISEPtr_t ptr(new EET_PIECEWISE(problem, roadmap));
  ptr->init(ptr);
  return ptr;
}

void EET_PIECEWISE::tryConnectInitAndGoals() {}

void EET_PIECEWISE::startSolve() {
  // core::PathPlanner::startSolve();
  // problem().checkProblem ();
  if (!problem()->robot()) {
    std::string msg("No device in problem.");
    hppDout(error, msg);
    throw std::runtime_error(msg);
  }

  if (!problem()->initConfig()) {
    std::string msg("No init config in problem.");
    hppDout(error, msg);
    throw std::runtime_error(msg);
  }

  // Tag init and goal configurations in the roadmap
  roadmap()->resetGoalNodes();

  cout << typeid(problem()->steeringMethod()).name() << endl;

  SMPtr_t sm(HPP_DYNAMIC_PTR_CAST(SM_t, problem()->steeringMethod()));
  if (!sm)
    throw std::invalid_argument(
        "Steering method must be of type "
        "hpp::manipulation::steeringMethod::EET_PIECEWISE");

  if (!sm->constraints() || !sm->constraints()->configProjector())
    throw std::invalid_argument(
        "Steering method constraint has no ConfigProjector.");
  core::ConfigProjectorPtr_t constraints(sm->constraints()->configProjector());

  const constraints::ImplicitPtr_t& trajConstraint = sm->trajectoryConstraint();
  if (!trajConstraint)
    throw std::invalid_argument("EET_PIECEWISE has no trajectory constraint.");
  if (!sm->trajectory())
    throw std::invalid_argument("EET_PIECEWISE has no trajectory.");

  const core::NumericalConstraints_t& ncs = constraints->numericalConstraints();
  bool ok = false;
  for (std::size_t i = 0; i < ncs.size(); ++i) {
    if (ncs[i] == trajConstraint) {
      ok = true;
      break;  // Same pointer
    }
    // Here, we do not check the right hand side on purpose.
    // if (*ncs[i] == *trajConstraint) {
    if (ncs[i]->functionPtr() == trajConstraint->functionPtr() &&
        ncs[i]->comparisonType() == trajConstraint->comparisonType()) {
      ok = true;
      // TODO We should only modify the path constraint.
      // However, only the pointers to implicit constraints are copied
      // while we would like the implicit constraints to be copied as well.
      ncs[i]->rightHandSideFunction(sm->trajectory());
      break;  // logically identical
    }
  }
  if (!ok) {
    HPP_THROW(std::logic_error,
              "EET_PIECEWISE could not find "
              "constraint "
                  << trajConstraint->function());
  }
}

void EET_PIECEWISE::oneStep() {
  SMPtr_t sm(HPP_DYNAMIC_PTR_CAST(SM_t, problem()->steeringMethod()));
  if (!sm)
    throw std::invalid_argument(
        "Steering method must be of type "
        "hpp::manipulation::steeringMethod::EET_PIECEWISE");
  if (!sm->trajectoryConstraint())
    throw std::invalid_argument("EET_PIECEWISE has no trajectory constraint.");
  if (!sm->trajectory())
    throw std::invalid_argument("EET_PIECEWISE has no trajectory.");

  if (!sm->constraints() || !sm->constraints()->configProjector())
    throw std::invalid_argument(
        "Steering method constraint has no ConfigProjector.");
  core::ConfigProjectorPtr_t constraints(sm->constraints()->configProjector());

  core::ConfigValidationPtr_t cfgValidation(problem()->configValidations());
  core::PathValidationPtr_t pathValidation(problem()->pathValidation());
  core::ValidationReportPtr_t cfgReport;
  core::PathValidationReportPtr_t pathReport;

  core::interval_t timeRange(sm->timeRange());

  std::vector<core::Configuration_t> qs(
      configurations(*problem()->initConfig()));
  if (qs.empty()) {
    hppDout(info, "Failed to generate initial configs.");
    return;
  }

  // Generate a valid initial configuration.
  bool success = false;
  bool resetRightHandSide = true;
  std::size_t i;

  vector_t times(nDiscreteSteps_ + 1);
  matrix_t steps(problem()->robot()->configSize(), nDiscreteSteps_ + 1);

  times[0] = timeRange.first;
  for (int j = 1; j < nDiscreteSteps_; ++j)
    times[j] = timeRange.first +
               j * (timeRange.second - timeRange.first) / nDiscreteSteps_;
  times[nDiscreteSteps_] = timeRange.second;

  for (i = 0; i < qs.size(); ++i) {
    if (resetRightHandSide) {
      constraints->rightHandSideAt(times[0]);
      resetRightHandSide = false;
    }
    Configuration_t& q(qs[i]);
    if (!constraints->apply(q)) continue;
    if (!cfgValidation->validate(q, cfgReport)) continue;
    resetRightHandSide = true;

    steps.col(0) = q;
    success = true;
    for (int j = 1; j <= nDiscreteSteps_; ++j) {
      constraints->rightHandSideAt(times[j]);
      hppDout(info, "RHS: " << setpyformat
                            << constraints->rightHandSide().transpose());
      steps.col(j) = steps.col(j - 1);
      if (!constraints->apply(steps.col(j))) {
        hppDout(info, "Failed to generate destination config.\n"
                          << setpyformat << *constraints
                          << "\nq=" << one_line(q));
        success = false;
        break;
      }
    }
    if (!success) continue;
    success = false;

    if (!cfgValidation->validate(steps.col(nDiscreteSteps_), cfgReport)) {
      hppDout(info, "Destination config is in collision.");
      continue;
    }

    core::PathPtr_t path = sm->projectedPath(times, steps);
    if (!path) {
      hppDout(info, "Steering method failed.\n"
                        << setpyformat << "times: " << one_line(times) << '\n'
                        << "configs:\n"
                        << condensed(steps.transpose()) << '\n');
      continue;
    }

    core::PathPtr_t validPart;
    if (!pathValidation->validate(path, false, validPart, pathReport)) {
      hppDout(info, "Path is in collision.");
      continue;
    }

    roadmap()->initNode(make_shared<Configuration_t>(steps.col(0)));
    core::NodePtr_t init = roadmap()->initNode();
    core::NodePtr_t goal = roadmap()->addGoalNode(
        make_shared<Configuration_t>(steps.col(nDiscreteSteps_)));
    roadmap()->addEdge(init, goal, path);
    success = true;
    if (feasibilityOnly_) break;
  }
}

std::vector<core::Configuration_t> EET_PIECEWISE::configurations(
    const core::Configuration_t& q_init) {
  if (!ikSolverInit_) {
    std::vector<core::Configuration_t> configs(nRandomConfig_ + 1);
    configs[0] = q_init;
    for (int i = 1; i < nRandomConfig_ + 1; ++i)
      problem()->configurationShooter()->shoot(configs[i]);
    return configs;
  }

  // TODO Compute the target and call ikSolverInit_
  // See https://gepgitlab.laas.fr/airbus-xtct/hpp_airbus_xtct for an
  // example using IKFast.
  throw std::runtime_error(
      "Using an IkSolverInitialization is not implemented yet");
}

EET_PIECEWISE::EET_PIECEWISE(const core::ProblemConstPtr_t& problem)
    : core::PathPlanner(problem) {}

EET_PIECEWISE::EET_PIECEWISE(const core::ProblemConstPtr_t& problem,
                             const core::RoadmapPtr_t& roadmap)
    : core::PathPlanner(problem, roadmap) {}

void EET_PIECEWISE::checkFeasibilityOnly(bool enable) {
  feasibilityOnly_ = enable;
}

void EET_PIECEWISE::init(const EET_PIECEWISEWkPtr_t& weak) {
  core::PathPlanner::init(weak);
  weak_ = weak;
  nRandomConfig_ = 10;
  nDiscreteSteps_ = 1;
  feasibilityOnly_ = true;
}

typedef manipulation::steeringMethod::EET_HERMITE HSM_t;
typedef manipulation::steeringMethod::EET_HERMITEPtr_t HSMPtr_t;

EET_HERMITEPtr_t EET_HERMITE::create(const core::ProblemConstPtr_t& problem,
                                     const core::RoadmapPtr_t& roadmap) {
  value_type M(2);
  EET_HERMITEPtr_t ptr(new EET_HERMITE(problem, M));
  ptr->init(ptr);
  return ptr;
}

PathVectorPtr_t EET_HERMITE::solve() {
  namespace bpt = boost::posix_time;

  interrupt_ = false;
  unsigned long int nIter(0);
  bpt::ptime timeStart(bpt::microsec_clock::universal_time());
  startSolve();
  // We choose to stop if a direct path solves the problem.
  // We could also respect the stopWhenLimitReached_ attribute.
  // It is ambiguous what should be done as it is case dependent.
  // If the intent is to build a roadmap, then we should not stop.
  // If the intent is to solve a optimal planning problem, then we should stop.
  problem_solved = false;

  if (interrupt_) throw hpp::core::path_planning_failed("Interruption");
  while (!problem_solved) {
    // Check limits
    std::ostringstream oss;
    if (maxIterations_ != uint_infty && nIter >= maxIterations_) {
      if (!stopWhenProblemIsSolved_ && problem_solved) break;
      oss << "Maximal number of iterations reached: " << maxIterations_;
      throw hpp::core::path_planning_failed(oss.str().c_str());
    }
    bpt::ptime timeStop(bpt::microsec_clock::universal_time());
    value_type elapsed_ms =
        static_cast<value_type>((timeStop - timeStart).total_milliseconds());
    if (elapsed_ms > timeOut_ * 1000) {
      if (!stopWhenProblemIsSolved_ && problem_solved) break;
      oss << "time out (" << timeOut_ << "s) reached after "
          << elapsed_ms * 1e-3 << "s";
      throw hpp::core::path_planning_failed(oss.str().c_str());
    }

    // Execute one step
    hppStartBenchmark(ONE_STEP);
    oneStep();
    hppStopBenchmark(ONE_STEP);
    hppDisplayBenchmark(ONE_STEP);

    // Check if problem is solved.
    ++nIter;
    if (interrupt_) throw("Interruption");
  }
  PathVectorPtr_t planned = final_answer;
  return planned;
}

void EET_HERMITE::tryConnectInitAndGoals() {}

void EET_HERMITE::startSolve() {
  if (!problem()->robot()) {
    std::string msg("No device in problem.");
    hppDout(error, msg);
    throw std::runtime_error(msg);
  }

  if (!problem()->initConfig()) {
    std::string msg("No init config in problem.");
    hppDout(error, msg);
    throw std::runtime_error(msg);
  }

  cout << typeid(problem()->steeringMethod()).name() << endl;

  HSMPtr_t sm(HPP_DYNAMIC_PTR_CAST(HSM_t, problem()->steeringMethod()));
  if (!sm)
    throw std::invalid_argument(
        "Steering method must be of type "
        "hpp::core::steeringMethod::Hermite");

  if (!sm->constraints() || !sm->constraints()->configProjector())
    throw std::invalid_argument(
        "Steering method constraint has no ConfigProjector.");
  core::ConfigProjectorPtr_t constraints(sm->constraints()->configProjector());

  const constraints::ImplicitPtr_t& trajConstraint = sm->trajectoryConstraint();
  if (!trajConstraint)
    throw std::invalid_argument("EET_HERMITE has no trajectory constraint.");
  if (!sm->trajectory())
    throw std::invalid_argument("EET_HERMITE has no trajectory.");

  const core::NumericalConstraints_t& ncs = constraints->numericalConstraints();
  bool ok = false;
  for (std::size_t i = 0; i < ncs.size(); ++i) {
    if (ncs[i] == trajConstraint) {
      ok = true;
      break;  // Same pointer
    }
    // Here, we do not check the right hand side on purpose.
    // if (*ncs[i] == *trajConstraint) {
    if (ncs[i]->functionPtr() == trajConstraint->functionPtr() &&
        ncs[i]->comparisonType() == trajConstraint->comparisonType()) {
      ok = true;
      // TODO We should only modify the path constraint.
      // However, only the pointers to implicit constraints are copied
      // while we would like the implicit constraints to be copied as well.
      ncs[i]->rightHandSideFunction(sm->trajectory());
      break;  // logically identical
    }
  }
  if (!ok) {
    HPP_THROW(std::logic_error,
              "EET_HERMITE could not find "
              "constraint "
                  << trajConstraint->function());
  }
}

void EET_HERMITE::oneStep() {
  HSMPtr_t sm(HPP_DYNAMIC_PTR_CAST(HSM_t, problem()->steeringMethod()));
  if (!sm)
    throw std::invalid_argument(
        "Steering method must be of type "
        "hpp::manipulation::steeringMethod::EET_HERMITE");
  if (!sm->trajectoryConstraint())
    throw std::invalid_argument("EET_HERMITE has no trajectory constraint.");
  if (!sm->trajectory())
    throw std::invalid_argument("EET_HERMITE has no trajectory.");

  if (!sm->constraints() || !sm->constraints()->configProjector())
    throw std::invalid_argument(
        "Steering method constraint has no ConfigProjector.");
  core::ConfigProjectorPtr_t constraints(sm->constraints()->configProjector());

  core::ConfigValidationPtr_t cfgValidation(problem()->configValidations());
  core::PathValidationPtr_t pathValidation(problem()->pathValidation());
  core::ValidationReportPtr_t cfgReport;
  core::PathValidationReportPtr_t pathReport;

  core::interval_t timeRange(sm->timeRange());

  std::vector<core::Configuration_t> qs(
      configurations(*problem()->initConfig()));
  if (qs.empty()) {
    hppDout(info, "Failed to generate initial configs.");
    return;
  }

  // Generate a valid initial configuration.
  bool success = false;
  bool resetRightHandSide = true;
  std::size_t i;

  vector_t times(nDiscreteSteps_ + 1);
  matrix_t steps(problem()->robot()->configSize(), nDiscreteSteps_ + 1);

  times[0] = timeRange.first;
  for (int j = 1; j < nDiscreteSteps_; ++j)
    times[j] = timeRange.first +
               j * (timeRange.second - timeRange.first) / nDiscreteSteps_;
  times[nDiscreteSteps_] = timeRange.second;

  for (i = 0; i < qs.size(); ++i) {
    if (resetRightHandSide) {
      constraints->rightHandSideAt(times[0]);
      resetRightHandSide = false;
    }
    Configuration_t& q(qs[i]);
    if (!constraints->apply(q)) continue;
    if (!cfgValidation->validate(q, cfgReport)) continue;
    resetRightHandSide = true;

    steps.col(0) = q;
    success = true;
    for (int j = 1; j <= nDiscreteSteps_; ++j) {
      constraints->rightHandSideAt(times[j]);
      hppDout(info, "RHS: " << setpyformat
                            << constraints->rightHandSide().transpose());
      steps.col(j) = steps.col(j - 1);
      if (!constraints->apply(steps.col(j))) {
        hppDout(info, "Failed to generate destination config.\n"
                          << setpyformat << *constraints
                          << "\nq=" << one_line(q));
        success = false;
        break;
      }
    }
    if (!success) continue;
    success = false;

    if (!cfgValidation->validate(steps.col(nDiscreteSteps_), cfgReport)) {
      hppDout(info, "Destination config is in collision.");
      continue;
    }

    core::PathPtr_t path = sm->projectedPath(times, steps);
    if (!path) {
      hppDout(info, "Steering method failed.\n"
                        << setpyformat << "times: " << one_line(times) << '\n'
                        << "configs:\n"
                        << condensed(steps.transpose()) << '\n');
      continue;
    }

    success = false;
    core::pathProjector::RecursiveHermitePtr_t recursor(
        core::pathProjector::RecursiveHermite::create(problem(), M));

    core::PathPtr_t answer(hpp::core::PathVector::create(
        problem()->robot()->configSize(), path->outputDerivativeSize()));

    if (!recursor->impl_apply(path, answer)) break;
    cout << "passed 2 \n\n\n" << endl;

    final_answer = HPP_DYNAMIC_PTR_CAST(core::PathVector, answer);
    problem_solved = true;
    success = true;
    if (feasibilityOnly_) break;
  }
}

std::vector<core::Configuration_t> EET_HERMITE::configurations(
    const core::Configuration_t& q_init) {
  if (!ikSolverInit_) {
    std::vector<core::Configuration_t> configs(nRandomConfig_ + 1);
    configs[0] = q_init;
    for (int i = 1; i < nRandomConfig_ + 1; ++i)
      problem()->configurationShooter()->shoot(configs[i]);
    return configs;
  }

  // TODO Compute the target and call ikSolverInit_
  // See https://gepgitlab.laas.fr/airbus-xtct/hpp_airbus_xtct for an
  // example using IKFast.
  throw std::runtime_error(
      "Using an IkSolverInitialization is not implemented yet");
}

EET_HERMITE::EET_HERMITE(const core::ProblemConstPtr_t& problem, value_type& i)
    : core::PathPlanner(problem), M(i) {}

void EET_HERMITE::checkFeasibilityOnly(bool enable) {
  feasibilityOnly_ = enable;
}

void EET_HERMITE::init(const EET_HERMITEWkPtr_t& weak) {
  core::PathPlanner::init(weak);
  weak_ = weak;
  nRandomConfig_ = 10;
  nDiscreteSteps_ = 1;
  feasibilityOnly_ = true;
}

}  // namespace pathPlanner
}  // namespace manipulation
}  // namespace hpp
