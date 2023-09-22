// Copyright (c) 2023 CNRS
// Authors: Florent Lamiraux
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

#include <hpp/core/collision-validation.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/core/config-validations.hh>
#include <hpp/core/constraint-set.hh>
#include <hpp/core/distance/reeds-shepp.hh>
#include <hpp/core/joint-bound-validation.hh>
#include <hpp/core/path-optimization/simple-time-parameterization.hh>
#include <hpp/core/path-optimizer.hh>
#include <hpp/core/path-planner/bi-rrt-star.hh>
#include <hpp/core/path-projector.hh>
#include <hpp/core/path-validation-report.hh>
#include <hpp/core/path-validation.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/steering-method/reeds-shepp.hh>
#include <hpp/manipulation/graph/edge.hh>
#include <hpp/manipulation/graph/graph.hh>
#include <hpp/manipulation/path-planner/transition-planner.hh>
#include <hpp/manipulation/problem.hh>
#include <hpp/manipulation/roadmap.hh>

namespace hpp {
namespace manipulation {
namespace pathPlanner {
TransitionPlannerPtr_t TransitionPlanner::createWithRoadmap(
    const core::ProblemConstPtr_t& problem, const core::RoadmapPtr_t& roadmap) {
  TransitionPlanner* ptr(new TransitionPlanner(problem, roadmap));
  TransitionPlannerPtr_t shPtr(ptr);
  ptr->init(shPtr);
  return shPtr;
}

void TransitionPlanner::startSolve() {
  // Check that edge has been selected
  // Initialize the planner
  if (!innerProblem_->constraints() ||
      !innerProblem_->constraints()->configProjector())
    throw std::logic_error(
        "TransitionPlanner::startSolve: inner problem has"
        " no constraints. You probably forgot to select "
        "the transition.");
  innerProblem_->constraints()->configProjector()->rightHandSideFromConfig(
      *(innerProblem_->initConfig()));
  // Forward maximal number of iterations to inner planner
  innerPlanner_->maxIterations(this->maxIterations());
  // Forward timeout to inner planner
  innerPlanner_->timeOut(this->timeOut());

  // Call parent implementation
  core::PathPlanner::startSolve();
}

void TransitionPlanner::oneStep() { innerPlanner_->oneStep(); }

core::PathVectorPtr_t TransitionPlanner::planPath(const Configuration_t qInit,
                                                  matrixIn_t qGoals,
                                                  bool resetRoadmap) {
  ConfigProjectorPtr_t configProjector(
      innerProblem_->constraints()->configProjector());
  if (configProjector) {
    configProjector->rightHandSideFromConfig(qInit);
  }
  ConfigurationPtr_t q(new Configuration_t(qInit));
  innerProblem_->initConfig(q);
  innerProblem_->resetGoalConfigs();
  for (size_type r = 0; r < qGoals.rows(); ++r) {
    ConfigurationPtr_t q(new Configuration_t(qGoals.row(r)));
    if (!configProjector->isSatisfied(*q)) {
      std::ostringstream os;
      os << "hpp::manipulation::TransitionPlanner::computePath: "
         << "goal configuration at rank " << r
         << " does not satisfy the leaf constraint.";
      throw std::logic_error(os.str().c_str());
    }
    innerProblem_->addGoalConfig(q);
  }
  if (resetRoadmap) {
    roadmap()->clear();
  }
  PathVectorPtr_t path = innerPlanner_->solve();
  path = optimizePath(path);
  return path;
}

core::PathPtr_t TransitionPlanner::directPath(const Configuration_t& q1,
                                              const Configuration_t& q2,
                                              bool validate, bool& success,
                                              std::string& status) {
  core::PathPtr_t res(innerProblem_->steeringMethod()->steer(q1, q2));
  if (!res) {
    success = false;
    status = std::string("Steering method failed");
    return res;
  }
  status = std::string("");
  core::PathProjectorPtr_t pathProjector(innerProblem_->pathProjector());
  bool success1 = true, success2 = true;
  core::PathPtr_t projectedPath = res;
  if (pathProjector) {
    success1 = pathProjector->apply(res, projectedPath);
    if (!success1) {
      status += std::string("Failed to project the path. ");
    }
  }
  core::PathPtr_t validPart = projectedPath;
  core::PathValidationPtr_t pathValidation(innerProblem_->pathValidation());
  if (pathValidation && validate) {
    core::PathValidationReportPtr_t report;
    success2 =
        pathValidation->validate(projectedPath, false, validPart, report);
    if (!success2) {
      status += std::string("Failed to validate the path. ");
    }
  }
  success = success1 && success2;
  return validPart;
}

core::PathVectorPtr_t TransitionPlanner::optimizePath(const PathPtr_t& path) {
  PathVectorPtr_t pv(HPP_DYNAMIC_PTR_CAST(PathVector, path));
  if (!pv) {
    pv = core::PathVector::create(path->outputSize(),
                                  path->outputDerivativeSize());
    pv->appendPath(path);
  }
  for (auto po : pathOptimizers_) {
    pv = po->optimize(pv);
  }
  return pv;
}

core::PathVectorPtr_t TransitionPlanner::timeParameterization(
    const PathVectorPtr_t& path) {
  core::PathOptimizerPtr_t tp(
      core::pathOptimization::SimpleTimeParameterization::create(
          innerProblem_));
  return tp->optimize(path);
}

void TransitionPlanner::setEdge(std::size_t id) {
  ProblemConstPtr_t p(HPP_DYNAMIC_PTR_CAST(const Problem, problem()));
  assert(p);
  graph::GraphComponentPtr_t comp(p->constraintGraph()->get(id).lock());
  graph::EdgePtr_t edge(HPP_DYNAMIC_PTR_CAST(graph::Edge, comp));
  if (!edge) {
    std::ostringstream os;
    os << "hpp::manipulation::pathPlanner::TransitionPlanner::setEdge: index "
       << id << " does not correspond to any edge of the constraint graph.";
    throw std::logic_error(os.str().c_str());
  }
  innerProblem_->constraints(edge->pathConstraint());
  innerProblem_->pathValidation(edge->pathValidation());
  innerProblem_->steeringMethod(edge->steeringMethod());
}

void TransitionPlanner::setReedsAndSheppSteeringMethod(double turningRadius) {
  core::JointPtr_t root(innerProblem_->robot()->rootJoint());
  core::SteeringMethodPtr_t sm(core::steeringMethod::ReedsShepp::create(
      innerProblem_, turningRadius, root, root));
  core::DistancePtr_t dist(core::distance::ReedsShepp::create(innerProblem_));
  innerProblem_->steeringMethod(sm);
  innerProblem_->distance(dist);
}

void TransitionPlanner::pathProjector(const PathProjectorPtr_t pathProjector) {
  innerProblem_->pathProjector(pathProjector);
}

void TransitionPlanner::clearPathOptimizers() { pathOptimizers_.clear(); }

/// Add a path optimizer
void TransitionPlanner::addPathOptimizer(
    const core::PathOptimizerPtr_t& pathOptimizer) {
  pathOptimizers_.push_back(pathOptimizer);
}

void TransitionPlanner::setParameter(const std::string& key,
                                     const core::Parameter& value) {
  innerProblem_->setParameter(key, value);
}

TransitionPlanner::TransitionPlanner(const core::ProblemConstPtr_t& problem,
                                     const core::RoadmapPtr_t& roadmap)
    : PathPlanner(problem, roadmap) {
  ProblemConstPtr_t p(HPP_DYNAMIC_PTR_CAST(const Problem, problem));
  if (!p)
    throw std::invalid_argument(
        "The problem should be of type hpp::manipulation::Problem.");
  // create the inner problem
  innerProblem_ = core::Problem::create(p->robot());
  // Pass parameters from manipulation problem
  std::vector<std::string> keys(
      p->parameters.getKeys<std::vector<std::string> >());
  for (auto k : keys) {
    innerProblem_->setParameter(k, p->parameters.get(k));
  }
  // Initialize config validations
  innerProblem_->clearConfigValidations();
  innerProblem_->configValidations()->add(
      hpp::core::CollisionValidation::create(p->robot()));
  innerProblem_->configValidations()->add(
      hpp::core::JointBoundValidation::create(p->robot()));
  // Add obstacles to inner problem
  innerProblem_->collisionObstacles(p->collisionObstacles());
  // Create default path planner
  innerPlanner_ = hpp::core::pathPlanner::BiRrtStar::createWithRoadmap(
      innerProblem_, roadmap);
}

void TransitionPlanner::init(TransitionPlannerWkPtr_t weak) {
  core::PathPlanner::init(weak);
  weakPtr_ = weak;
}

}  // namespace pathPlanner
}  // namespace manipulation
}  // namespace hpp
