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

#ifndef HPP_MANIPULATION_PATH_PLANNER_TRANSITION_PLANNER_HH
#define HPP_MANIPULATION_PATH_PLANNER_TRANSITION_PLANNER_HH

#include <hpp/core/path-planner.hh>
#include <hpp/manipulation/fwd.hh>

namespace hpp {
namespace manipulation {
namespace pathPlanner {

/// \addtogroup path_planner
/// \{

/// Plan paths in a leaf of a transition
///
/// In many manipulation applications, the sequence of actions is knwown in
/// advance or computed by a task planner. There is a need then to connect
/// configurations that lie on the same leaf of a transition. This class
/// performs this computation.
///
/// The constraint graph is stored in the Problem instance of the planner.
/// To select the transition, call method \link TransitionPlanner::setEdge
/// setEdge \endlink with the index of the transition.
///
/// At construction, a core::Problem instance is created, as well as a
/// core::PathPlanner instance. They are respectively called the inner problem
/// and the inner planner.
///
/// The leaf of the transition is defined by the initial configuration passed
/// to method \link TransitionPlanner::planPath planPath \endlink.
/// The right hand side of the inner problem constraints is initialized with
/// this configuration.
///
/// The class stores path optimizers that are called when invoking method
/// \link TransitionPlanner::optimizePath optimizePath \endlink.
///
/// Method \link TransitionPlanner::timeParameterization timeParameterization
/// \endlink computes a time parameterization of a given path.
class HPP_MANIPULATION_DLLAPI TransitionPlanner : public core::PathPlanner {
 public:
  typedef core::PathPlannerPtr_t PathPlannerPtr_t;
  typedef core::PathProjectorPtr_t PathProjectorPtr_t;
  typedef core::PathPtr_t PathPtr_t;
  typedef core::PathOptimizerPtr_t PathOptimizerPtr_t;
  typedef core::PathVector PathVector;
  typedef core::PathVectorPtr_t PathVectorPtr_t;
  typedef core::Parameter Parameter;

  /// Create instance and return share pointer
  static TransitionPlannerPtr_t createWithRoadmap(
      const core::ProblemConstPtr_t& problem,
      const core::RoadmapPtr_t& roadmap);
  /// Get the inner planner
  PathPlannerPtr_t innerPlanner() const { return innerPlanner_; }
  /// Set the inner planner
  void innerPlanner(const PathPlannerPtr_t& planner) {
    innerPlanner_ = planner;
  }
  /// Get the inner problem
  core::ProblemPtr_t innerProblem() const { return innerProblem_; }
  /// Initialize path planning
  /// Set right hand side of problem constraints with initial configuration
  virtual void startSolve();

  /// One step of the planner
  /// Calls the same method of the internally stored planner
  virtual void oneStep();

  /// Solve the problem defined by input configurations
  /// \param qInit initial configuration,
  /// \param qGoals, goal configurations,
  /// \param resetRoadmap whether to reset the roadmap
  PathVectorPtr_t planPath(const Configuration_t qInit, matrixIn_t qGoals,
                           bool resetRoadmap);
  /// Call the steering method between two configurations
  /// \param q1, q2 the start and end configurations,
  /// \param validate whether resulting path should be tested for collision
  /// \retval success True if path has been computed and validated
  ///         successfully
  /// \retval status a message in case of failure.
  /// If a path projector has been selected, the path is tested for
  /// continuity.
  PathPtr_t directPath(ConfigurationIn_t q1, ConfigurationIn_t q2,
                       bool validate, bool& success, std::string& status);
  /// Optimize path using the selected path optimizers
  /// \param path input path
  /// \return optimized path
  PathVectorPtr_t optimizePath(const PathPtr_t& path);

  /// Compute time parameterization of path
  /// \param path input path
  /// \return time parameterized trajectory
  /// Uses core::pathOptimization::SimpleTimeParameterization.
  PathVectorPtr_t timeParameterization(const PathVectorPtr_t& path);

  /// Set transition along which we wish to plan a path
  /// \param id index of the edge in the constraint graph
  void setEdge(std::size_t id);

  /// Create a Reeds and Shepp steering method and path it to the problem.
  void setReedsAndSheppSteeringMethod(double turningRadius);

  /// Set the path projector
  void pathProjector(const PathProjectorPtr_t pathProjector);

  /// Clear path optimizers
  void clearPathOptimizers();

  /// Add a path optimizer
  void addPathOptimizer(const PathOptimizerPtr_t& pathOptimizer);

  /// Set parameter to the inner problem
  void setParameter(const std::string& key, const Parameter& value);

 protected:
  /// Constructor
  /// Cast problem into manipulation::Problem and store shared pointer
  ///
  /// Create an inner problem and forward the following elements of the input
  /// problem to the inner problem:
  /// \li parameters,
  /// \li collision obstacles.
  /// Add instances of core::CollisionValidation and
  /// core::JointBoundValidation to the inner problem.
  TransitionPlanner(const core::ProblemConstPtr_t& problem,
                    const core::RoadmapPtr_t& roadmap);
  /// store weak pointer to itself
  void init(TransitionPlannerWkPtr_t weak);

 private:
  /// Pointer to the problem of the inner planner
  core::ProblemPtr_t innerProblem_;
  /// Pointer to the inner path planner
  PathPlannerPtr_t innerPlanner_;
  /// Vector of optimizers to call after solve
  std::vector<PathOptimizerPtr_t> pathOptimizers_;
  /// weak pointer to itself
  TransitionPlannerWkPtr_t weakPtr_;
};  // class TransitionPlanner
}  // namespace pathPlanner
}  // namespace manipulation
}  // namespace hpp
#endif  // HPP_MANIPULATION_PATH_PLANNER_TRANSITION_PLANNER_HH
