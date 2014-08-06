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

#include <hpp/util/pointer.hh>
#include <hpp/util/debug.hh>
#include <hpp/model/gripper.hh>
#include <hpp/core/roadmap.hh>

#include "hpp/manipulation/object.hh"
#include "hpp/manipulation/robot.hh"
#include "hpp/manipulation/graph/graph.hh"
#include "hpp/manipulation/manipulation-planner.hh"
#include "hpp/manipulation/problem.hh"

#include "hpp/manipulation/problem-solver.hh"

namespace hpp {
  namespace manipulation {
    void ProblemSolver::buildCompositeRobot (const std::string& robotName,
					     const Names_t& robotNames)
    {
      Devices_t robots;
      Objects_t objects;
      Names_t::const_iterator itName = robotNames.begin ();
      for (;
	   itName != robotNames.end (); ++itName) {
	const DevicePtr_t& rob (robotsAndObjects_ [*itName]);
	ObjectPtr_t object = HPP_DYNAMIC_PTR_CAST (Object, rob);
	if (object) {
	  objects.push_back (object);
	} else {
	  robots.push_back (rob);
	}
      }
      RobotPtr_t composite (Robot::create (robotName, robots, objects));
      robot (composite);
      hppDout (info, *composite);
    }

    DevicePtr_t ProblemSolver::robot (const std::string& name) const
    {
      RobotsandObjects_t::const_iterator it =
	robotsAndObjects_.find (name);
      if (it == robotsAndObjects_.end ()) {
	throw std::runtime_error ("No robot nor object with this name");
      }
      return it->second;
    }

    ObjectPtr_t ProblemSolver::object (const std::string& name) const
    {
      RobotsandObjects_t::const_iterator it = robotsAndObjects_.find (name);
      if (it == robotsAndObjects_.end ()) {
	throw std::runtime_error ("No robot nor object with this name");
      }
      ObjectPtr_t object = HPP_DYNAMIC_PTR_CAST (Object, it->second);
      if (!object) {
	throw std::runtime_error (name + std::string (" is not an object"));
      }
      return object;
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
    }

    void ProblemSolver::constraintGraph (const graph::GraphPtr_t& graph)
    {
      constraintGraph_ = graph;
    }

    graph::GraphPtr_t ProblemSolver::constraintGraph () const
    {
      return constraintGraph_;
    }

    LockedDofPtr_t ProblemSolver::lockedDofConstraint (const std::string& name) const
    {
      LockedDofConstraintMap_t::const_iterator it =
	lockedDofConstraintMap_.find (name);
      if (it == lockedDofConstraintMap_.end ()) {
	throw std::runtime_error ("No LockedDof constraint with this name");
      }
      return it->second;
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
             itGrasp != graspsMap.end() ; itGrasp++) {
        GraspPtr_t grasp = itGrasp->second;
        GripperPtr_t gripper = grasp->first;
        HandlePtr_t handle = grasp->second;
        JointPtr_t joint = handle->joint();
        model::JointVector_t joints = gripper->getDisabledCollisions();
        for (model::JointVector_t::iterator itJoint = joints.begin() ;
               itJoint != joints.end() ; itJoint++ ) {
          robot()->addCollisionPairs(joint, *itJoint, hpp::model::COLLISION);
          robot()->addCollisionPairs(joint, *itJoint, hpp::model::DISTANCE);
        }
      }
    }

    void ProblemSolver::addConstraintToConfigProjector (
                          const std::string& constraintName,
                          const DifferentiableFunctionPtr_t& constraint)
    {
      core::ProblemSolver::addConstraintToConfigProjector(constraintName,
                                                          constraint);
      if ( grasp(constraint) ) {
        GripperPtr_t gripper = grasp(constraint)->first;
        HandlePtr_t handle = grasp(constraint)->second;
        JointPtr_t joint1 = handle->joint();
        model::JointVector_t joints = gripper->getDisabledCollisions();
        for (model::JointVector_t::iterator itJoint = joints.begin() ;
               itJoint != joints.end() ; itJoint++ ) {
          robot()->removeCollisionPairs(joint1, *itJoint,
                                        hpp::model::COLLISION);
          robot()->removeCollisionPairs(joint1, *itJoint,
                                        hpp::model::DISTANCE);
        }
      }
    }
  } // namespace manipulation
} // namespace hpp
