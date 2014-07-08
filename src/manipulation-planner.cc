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

#include "hpp/manipulation/manipulation-planner.hh"

namespace hpp {
  namespace manipulation {
    PlannerPtr_t ManipulationPlanner::create (const core::Problem& problem,
        const core::RoadmapPtr_t& roadmap)
    {
      ManipulationPlanner* ptr = new ManipulationPlanner (problem, roadmap);
      PlannerPtr_t shPtr (ptr);
      ptr->init (shPtr);
      return shPtr;
    }

    void ManipulationPlanner::oneStep ()
    {
    }

    ManipulationPlanner::ManipulationPlanner (const core::Problem& problem,
        const core::RoadmapPtr_t& roadmap) :
      core::PathPlanner (problem, roadmap), shooter_ (problem.robot ())
    {}

    void ManipulationPlanner::init (const PlannerWkPtr_t& weak)
    {
      core::PathPlanner::init (weak);
      weakPtr_ = weak;
    }
  } // namespace manipulation
} // namespace hpp
