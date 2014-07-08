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

#ifndef HPP_MANIPULATION_MANIPULATION_PLANNER_HH
# define HPP_MANIPULATION_MANIPULATION_PLANNER_HH

#include <hpp/core/path-planner.hh>
#include <hpp/core/basic-configuration-shooter.hh>
#include <hpp/core/roadmap.hh>

#include "hpp/manipulation/graph/fwd.hh"
#include "hpp/manipulation/fwd.hh"


namespace hpp {
  namespace manipulation {
    class HPP_MANIPULATION_DLLAPI ManipulationPlanner:
      public hpp::core::PathPlanner
    {
      public:
        /// Create an instance and return a shared pointer to the instance
        static ManipulationPlannerPtr_t create (const core::Problem& problem,
            const core::RoadmapPtr_t& roadmap);

        /// One step of extension.
        /// 
        /// A set of constraints is choosen using the graph of constraints.
        /// A constraint extension is done using a choosen set.
        ///
        virtual void oneStep ();

      protected:
        /// Protected constructor
        ManipulationPlanner (const Problem& problem,
            const core::RoadmapPtr_t& roadmap);

        /// Store weak pointer to itself
        void init (const ManipulationPlannerWkPtr_t& weak);

      private:
        /// Configuration shooter to uniformly shoot random configurations
        core::BasicConfigurationShooter shooter_;
        /// weak pointer to itself
        ManipulationPlannerWkPtr_t weakPtr_;
    }
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_MANIPULATION_PLANNER_HH
