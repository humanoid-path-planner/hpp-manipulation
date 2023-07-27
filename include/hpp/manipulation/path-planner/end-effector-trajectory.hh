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

#ifndef HPP_MANIPULATION_PATH_PLANNER_END_EFFECTOR_TRAJECTORY_HH
#define HPP_MANIPULATION_PATH_PLANNER_END_EFFECTOR_TRAJECTORY_HH

#include <hpp/core/path-planner.hh>
#include <hpp/manipulation/config.hh>
#include <hpp/manipulation/fwd.hh>
#include <hpp/pinocchio/frame.hh>
#include <pinocchio/spatial/se3.hpp>

namespace hpp {
namespace manipulation {
namespace pathPlanner {
class HPP_MANIPULATION_DLLAPI IkSolverInitialization {
 public:
  typedef std::vector<Configuration_t> Configurations_t;

  Configurations_t solve(vectorIn_t target) { return impl_solve(target); }

 protected:
  virtual Configurations_t impl_solve(vectorIn_t target) = 0;
};
typedef shared_ptr<IkSolverInitialization> IkSolverInitializationPtr_t;

HPP_PREDEF_CLASS(EndEffectorTrajectory);
typedef shared_ptr<EndEffectorTrajectory> EndEffectorTrajectoryPtr_t;

/// \addtogroup path_planning
/// \{

/// Plan a path for a robot with constrained trajectory of an end effector
///
/// This path planner only works with a steering method of type
/// steeringMethod::EndEffectorTrajectory. The steering
/// method defines the desired end-effector trajectory using a time-varying
/// constraint.
///
/// To plan a path between two configurations \c q_init and \c q_goal, the
/// configurations must satisfy the constraint at the beginning and at the
/// end of the definition interval respectively.
///
/// The interval of definition \f$[0,T]\f$ of the output path is defined by the
/// time-varying constraint of the steering method. This interval is uniformly
/// discretized in a number of samples that can be accessed using method \link
/// EndEffectorTrajectory::nDiscreteSteps nDiscreteSteps \endlink.
///
/// The path is planned by successively calling method \link
/// EndEffectorTrajectory::oneStep oneStep \endlink that performs the following
/// actions.
///   - A vector of configurations is produced by appending random
///     configurations to \c q_init. The number of random configurations can
///     be accessed by methods \link EndEffectorTrajectory::nRandomConfig
///     nRandomConfig \endlink.
///   - for each configuration in the vector,
///     - the initial configuration of the path is computed by projecting the
///       configuration on the constraint,
///     - the configuration at following samples is computed by projecting the
///       configuration at the previous sample using the time-varying
///       constraint.
///     - In case of failure
///        - in projecting a configuration or
///        - in validating the path for collision,
///       the loop restart with the next random configuration.
///
/// Note that continuity is not tested but enforced by projecting the
/// configuration of the previous sample to compute the configuration at
/// a given sample.
class HPP_MANIPULATION_DLLAPI EndEffectorTrajectory : public core::PathPlanner {
 public:
  /// Return shared pointer to new instance
  /// \param problem the path planning problem
  static EndEffectorTrajectoryPtr_t create(
      const core::ProblemConstPtr_t& problem);
  /// Return shared pointer to new instance
  /// \param problem the path planning problem
  /// \param roadmap previously built roadmap
  static EndEffectorTrajectoryPtr_t createWithRoadmap(
      const core::ProblemConstPtr_t& problem,
      const core::RoadmapPtr_t& roadmap);

  /// Initialize the problem resolution
  ///  \li call parent implementation
  ///  \li get number nodes in problem parameter map
  virtual void startSolve();

  /// One step of the algorithm
  virtual void oneStep();

  /// Get the number of random configurations shoot (after using init
  /// config) in order to generate the initial config of the final path.
  int nRandomConfig() const { return nRandomConfig_; }

  void nRandomConfig(int n) {
    assert(n >= 0);
    nRandomConfig_ = n;
  }

  /// Number of steps to generate goal config (successive projections).
  int nDiscreteSteps() const { return nDiscreteSteps_; }

  void nDiscreteSteps(int n) {
    assert(n > 0);
    nDiscreteSteps_ = n;
  }

  /// If enabled, only add one solution to the roadmap.
  /// Otherwise add all solution.
  void checkFeasibilityOnly(bool enable);

  bool checkFeasibilityOnly() const { return feasibilityOnly_; }

  void ikSolverInitialization(IkSolverInitializationPtr_t solver) {
    ikSolverInit_ = solver;
  }

  void tryConnectInitAndGoals();

 protected:
  /// Protected constructor
  /// \param problem the path planning problem
  EndEffectorTrajectory(const core::ProblemConstPtr_t& problem);
  /// Protected constructor
  /// \param problem the path planning problem
  /// \param roadmap previously built roadmap
  EndEffectorTrajectory(const core::ProblemConstPtr_t& problem,
                        const core::RoadmapPtr_t& roadmap);
  /// Store weak pointer to itself
  void init(const EndEffectorTrajectoryWkPtr_t& weak);

 private:
  std::vector<core::Configuration_t> configurations(
      const core::Configuration_t& q_init);

  /// Weak pointer to itself
  EndEffectorTrajectoryWkPtr_t weak_;
  /// Number of random config.
  int nRandomConfig_;
  /// Number of steps to generate goal config.
  int nDiscreteSteps_;
  /// Ik solver initialization. An external Ik solver can be plugged here.
  IkSolverInitializationPtr_t ikSolverInit_;
  /// Feasibility
  bool feasibilityOnly_;
};  // class EndEffectorTrajectory
// \}
}  // namespace pathPlanner
}  // namespace manipulation
}  // namespace hpp

#endif  // HPP_MANIPULATION_PATH_PLANNER_END_EFFECTOR_TRAJECTORY_HH
