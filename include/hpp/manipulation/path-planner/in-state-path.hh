// Copyright (c) 2021, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr),
//          Florent Lamiraux (florent.lamiraux@laas.fr)
//          Alexandre Thiault (athiault@laas.fr)
//
// This file is part of hpp-manipulation.
// hpp-manipulation is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-manipulation is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-manipulation. If not, see <http://www.gnu.org/licenses/>.

#ifndef HPP_MANIPULATION_PATH_PLANNER_IN_STATE_PATH_HH
#define HPP_MANIPULATION_PATH_PLANNER_IN_STATE_PATH_HH

#include <hpp/core/collision-validation.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/core/joint-bound-validation.hh>
#include <hpp/core/path-planner.hh>
#include <hpp/manipulation/config.hh>
#include <hpp/manipulation/fwd.hh>
#include <hpp/manipulation/problem.hh>
#include <hpp/manipulation/steering-method/graph.hh>

namespace hpp {
namespace manipulation {
namespace pathPlanner {
/// \addtogroup path_planner
/// \{

///
/// Path planner designed to build a path between two configurations
/// on the same leaf of a given state graph edge
class HPP_MANIPULATION_DLLAPI InStatePath : public core::PathPlanner {
 public:
  virtual ~InStatePath() {}

  static InStatePathPtr_t create(const core::ProblemConstPtr_t& problem);

  static InStatePathPtr_t createWithRoadmap(
      const core::ProblemConstPtr_t& problem,
      const core::RoadmapPtr_t& roadmap);

  InStatePathPtr_t copy() const;

  void maxIterPlanner(const unsigned long& maxiter);
  void timeOutPlanner(const double& timeout);
  void resetRoadmap(const bool& resetroadmap);
  void plannerType(const std::string& plannertype);
  void addOptimizerType(const std::string& opttype);
  void resetOptimizerTypes();

  /// Set the edge along which q_init and q_goal will be linked.
  /// Use setEdge before setInit and setGoal.
  void setEdge(const graph::EdgePtr_t& edge);
  void setInit(const ConfigurationPtr_t& q);
  void setGoal(const ConfigurationPtr_t& q);

  virtual void oneStep() {}
  virtual core::PathVectorPtr_t solve();

  const core::RoadmapPtr_t& leafRoadmap() const;
  void mergeLeafRoadmapWith(const core::RoadmapPtr_t& roadmap) const;

 protected:
  InStatePath(const core::ProblemConstPtr_t& problem,
              const core::RoadmapPtr_t& roadmap)
      : PathPlanner(problem),
        problem_(HPP_STATIC_PTR_CAST(const manipulation::Problem, problem)),
        leafRoadmap_(roadmap),
        constraints_(),
        weak_() {
    const core::DevicePtr_t& robot = problem_->robot();
    leafProblem_ = core::Problem::create(robot);
    leafProblem_->setParameter("kPRM*/numberOfNodes",
                               core::Parameter((size_type)2000));
    leafProblem_->clearConfigValidations();
    leafProblem_->addConfigValidation(core::CollisionValidation::create(robot));
    leafProblem_->addConfigValidation(
        core::JointBoundValidation::create(robot));
    for (const core::CollisionObjectPtr_t& obs :
         problem_->collisionObstacles()) {
      leafProblem_->addObstacle(obs);
    }
    leafProblem_->pathProjector(problem->pathProjector());
  }

  InStatePath(const InStatePath& other)
      : PathPlanner(other.problem_),
        problem_(other.problem_),
        leafProblem_(other.leafProblem_),
        leafRoadmap_(other.leafRoadmap_),
        constraints_(other.constraints_),
        weak_() {}

  void init(InStatePathWkPtr_t weak) { weak_ = weak; }

 private:
  // a pointer to the problem used to create the InStatePath instance
  ProblemConstPtr_t problem_;
  // a new problem created for this InStatePath instance
  core::ProblemPtr_t leafProblem_;
  core::RoadmapPtr_t leafRoadmap_;
  ConstraintSetPtr_t constraints_;

  double timeOutPathPlanning_ = 0;
  unsigned long maxIterPathPlanning_ = 0;
  bool resetRoadmap_ = true;
  std::string plannerType_ = "BiRRT*";
  std::vector<std::string> optimizerTypes_;

  /// Weak pointer to itself
  InStatePathWkPtr_t weak_;

};  // class InStatePath
/// \}

}  // namespace pathPlanner
}  // namespace manipulation
}  // namespace hpp

#endif  // HPP_MANIPULATION_STEERING_METHOD_CROSS_STATE_OPTIMIZATION_HH
