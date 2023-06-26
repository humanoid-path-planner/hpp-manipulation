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
#include <hpp/core/path-vector.hh>
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

HPP_PREDEF_CLASS(EET_PIECEWISE);
typedef shared_ptr<EET_PIECEWISE> EET_PIECEWISEPtr_t;
typedef hpp::core::PathVectorPtr_t PathVectorPtr_t;

class HPP_MANIPULATION_DLLAPI EET_PIECEWISE : public core::PathPlanner {
 public:
  /// Return shared pointer to new instance
  /// \param problem the path planning problem
  static EET_PIECEWISEPtr_t create(
      const core::ProblemConstPtr_t& problem);
  /// Return shared pointer to new instance
  /// \param problem the path planning problem
  /// \param roadmap previously built roadmap
  static EET_PIECEWISEPtr_t createWithRoadmap(
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
  EET_PIECEWISE(const core::ProblemConstPtr_t& problem);
  /// Protected constructor
  /// \param problem the path planning problem
  /// \param roadmap previously built roadmap
  EET_PIECEWISE(const core::ProblemConstPtr_t& problem,
                        const core::RoadmapPtr_t& roadmap);
  /// Store weak pointer to itself
  void init(const EET_PIECEWISEWkPtr_t& weak);

 private:
  std::vector<core::Configuration_t> configurations(
      const core::Configuration_t& q_init);

  /// Weak pointer to itself
  EET_PIECEWISEWkPtr_t weak_;
  /// Number of random config.
  int nRandomConfig_;
  /// Number of steps to generate goal config.
  int nDiscreteSteps_;
  /// Ik solver initialization. An external Ik solver can be plugged here.
  IkSolverInitializationPtr_t ikSolverInit_;
  /// Feasibility
  bool feasibilityOnly_;
};  // class EET_PIECEWISE

}  // namespace pathPlanner
}  // namespace manipulation
}  // namespace hpp

#endif  // HPP_MANIPULATION_PATH_PLANNER_END_EFFECTOR_TRAJECTORY_HH
