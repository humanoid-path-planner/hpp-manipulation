// Copyright (c) 2014, LAAS-CNRS
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
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

#include <hpp/util/assertion.hh>

#include "hpp/manipulation/robot.hh"
#include "hpp/manipulation/manipulation-planner.hh"

namespace hpp {
  namespace manipulation {
    ManipulationPlannerPtr_t ManipulationPlanner::create (const core::Problem& problem,
        const core::RoadmapPtr_t& roadmap)
    {
      ManipulationPlanner* ptr = new ManipulationPlanner (problem, roadmap);
      ManipulationPlannerPtr_t shPtr (ptr);
      ptr->init (shPtr);
      return shPtr;
    }

    void ManipulationPlanner::oneStep ()
    {
      RobotPtr_t robot = HPP_DYNAMIC_PTR_CAST(Robot, problem ().robot ());
      HPP_ASSERT(robot);

      // Pick a random node
      ConfigurationPtr_t q_rand = shooter_.shoot();

      // Extend each connected component
      for (core::ConnectedComponents_t::const_iterator itcc =
          roadmap ()->connectedComponents ().begin ();
          itcc != roadmap ()->connectedComponents ().end (); itcc++) {
        core::NodePtr_t q_new = extendConnectedComponent(*itcc, q_rand);
      }
    }

    core::NodePtr_t ManipulationPlanner::extendConnectedComponent(
        ConnectedComponentPtr_t connectedComponent,
        const ConfigurationPtr_t &q_rand)
    {
      // Find the nearest neighbor.
      core::value_type distance;
      core::NodePtr_t near = roadmap ()->nearestNode (q_rand, connectedComponent, distance);

      // Select next node in the constraint graph.
      //graph::Edges_t edge = selectNextState(near);
    }

    ManipulationPlanner::ManipulationPlanner (const core::Problem& problem,
        const core::RoadmapPtr_t& roadmap) :
      core::PathPlanner (problem, roadmap), shooter_ (problem.robot ())
    {}

    void ManipulationPlanner::init (const ManipulationPlannerWkPtr_t& weak)
    {
      core::PathPlanner::init (weak);
      weakPtr_ = weak;
    }
  } // namespace manipulation
} // namespace hpp
