// Copyright (c) 2014 CNRS
// Author: Florent Lamiraux
//
// This file is part of hpp-manipulation-corba.
// hpp-manipulation-corba is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-manipulation-corba is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-manipulation-corba.  If not, see
// <http://www.gnu.org/licenses/>.

#include "hpp/manipulation/problem-solver.hh"

#include <hpp/util/pointer.hh>
#include <hpp/util/debug.hh>

#include <hpp/model/gripper.hh>

#include <hpp/core/random-shortcut.hh>
#include <hpp/core/discretized-collision-checking.hh>
#include <hpp/core/path-optimization/partial-shortcut.hh>

#include "hpp/manipulation/device.hh"
#include "hpp/manipulation/handle.hh"
#include "hpp/manipulation/graph/graph.hh"
#include "hpp/manipulation/manipulation-planner.hh"
#include "hpp/manipulation/problem.hh"
#include "hpp/manipulation/roadmap.hh"
#include "hpp/manipulation/constraint-set.hh"
#include "hpp/manipulation/graph-optimizer.hh"
#include "hpp/manipulation/graph-path-validation.hh"
#include "hpp/manipulation/graph-node-optimizer.hh"

namespace hpp {
  namespace manipulation {
    namespace {
      struct PartialShortcutTrait :
        core::pathOptimization::PartialShortcutTrait {
          static bool removeLockedJoints () { return false; }
      };
    }

    std::ostream& operator<< (std::ostream& os, const Device& robot)
    {
      return robot.print (os);
    }

    ProblemSolver::ProblemSolver () :
      core::ProblemSolver (), robot_ (), problem_ (0x0), graspsMap_()
    {
      addPathPlannerType ("M-RRT", ManipulationPlanner::create);
      addPathValidationType ("Graph-discretized",
          GraphPathValidation::create <core::DiscretizedCollisionChecking>);
      addPathOptimizerType ("Graph-RandomShortcut",
          GraphOptimizer::create <core::RandomShortcut>);
      addPathOptimizerType ("PartialShortcut", core::pathOptimization::
          PartialShortcut::createWithTrait <PartialShortcutTrait>);
      addPathOptimizerType ("Graph-PartialShortcut",
          GraphOptimizer::create <core::pathOptimization::PartialShortcut>);
      pathPlannerType ("M-RRT");
      pathValidationType ("Graph-discretized", 0.05);
    }

    ProblemSolverPtr_t ProblemSolver::create ()
    {
      return ProblemSolverPtr_t (new ProblemSolver ());
    }

    void ProblemSolver::resetProblem ()
    {
      if (problem_)
        delete (problem_);
      initializeProblem (new Problem (robot_));
    }

    void ProblemSolver::initializeProblem (ProblemPtr_t problem)
    {
      problem_ = problem;
      core::ProblemSolver::initializeProblem (problem_);
      if (problem_->pathValidation ())
        problem_->pathValidation ()->constraintGraph (constraintGraph_);
    }

    void ProblemSolver::constraintGraph (const graph::GraphPtr_t& graph)
    {
      constraintGraph_ = graph;
      RoadmapPtr_t r = HPP_DYNAMIC_PTR_CAST (Roadmap, roadmap());
      if (r) r->constraintGraph (graph);
    }

    graph::GraphPtr_t ProblemSolver::constraintGraph () const
    {
      return constraintGraph_;
    }

    GraspPtr_t ProblemSolver::grasp (
                      const DifferentiableFunctionPtr_t& constraint) const
    {
      GraspsMap_t::const_iterator it =
	graspsMap_.find (constraint);
      if (it == graspsMap_.end ()) {
        GraspPtr_t grasp;
	return grasp;
      }
      return it->second;
    }

    void ProblemSolver::resetConstraints ()
    {
      if (robot_)
	constraints_ = ConstraintSet::create (robot_,
                                              "Default constraint set");
      GraspsMap_t graspsMap = grasps();
      for (GraspsMap_t::const_iterator itGrasp = graspsMap.begin();
             itGrasp != graspsMap.end(); ++itGrasp) {
        GraspPtr_t grasp = itGrasp->second;
        GripperPtr_t gripper = grasp->first;
        HandlePtr_t handle = grasp->second;
        JointPtr_t joint = handle->joint();
        model::JointVector_t joints = gripper->getDisabledCollisions();
        for (model::JointVector_t::iterator itJoint = joints.begin() ;
               itJoint != joints.end(); ++itJoint) {
          robot()->addCollisionPairs(joint, *itJoint, hpp::model::COLLISION);
          robot()->addCollisionPairs(joint, *itJoint, hpp::model::DISTANCE);
        }
      }
    }

    void ProblemSolver::addFunctionToConfigProjector
    (const std::string& constraintName, const std::string& functionName)
    {
      core::ProblemSolver::addFunctionToConfigProjector (constraintName,
                                                         functionName);
      DifferentiableFunctionPtr_t constraint =
	numericalConstraint (functionName);
      if (GraspPtr_t g = grasp (constraint)) {
        GripperPtr_t gripper = g->first;
        HandlePtr_t handle = g->second;
        JointPtr_t joint1 = handle->joint();
        model::JointVector_t joints = gripper->getDisabledCollisions();
        for (model::JointVector_t::iterator itJoint = joints.begin() ;
               itJoint != joints.end(); ++itJoint++) {
          robot()->removeCollisionPairs(joint1, *itJoint,
                                        hpp::model::COLLISION);
          robot()->removeCollisionPairs(joint1, *itJoint,
                                        hpp::model::DISTANCE);
        }
      }
    }

    void ProblemSolver::resetRoadmap ()
    {
      if (!problem ())
        throw std::runtime_error ("The problem is not defined.");
      RoadmapPtr_t r (Roadmap::create (problem ()->distance (), problem ()->robot ()));
      if (constraintGraph_) r->constraintGraph (constraintGraph_);
      roadmap (r);
    }
  } // namespace manipulation
} // namespace hpp
