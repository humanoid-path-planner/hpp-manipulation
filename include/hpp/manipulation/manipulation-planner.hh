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

#include <hpp/model/configuration.hh>
#include <hpp/core/basic-configuration-shooter.hh>
#include <hpp/core/path-planner.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/roadmap.hh>

#include "hpp/manipulation/config.hh"
#include "hpp/manipulation/graph/fwd.hh"
#include "hpp/manipulation/graph/graph.hh"
#include "hpp/manipulation/fwd.hh"

namespace hpp {
  namespace manipulation {
    class HPP_MANIPULATION_DLLAPI ManipulationPlanner : public hpp::core::PathPlanner
    {
      public:
        /// Create an instance and return a shared pointer to the instance
        static ManipulationPlannerPtr_t create (const core::Problem& problem,
            const core::RoadmapPtr_t& roadmap);

        /// Set the constraint graph
        void constraintGraph (const graph::GraphPtr_t& graph)
        {
          constraintGraph_ = graph;
        }

        /// One step of extension.
        /// 
        /// A set of constraints is choosen using the graph of constraints.
        /// A constraint extension is done using a choosen set.
        ///
        virtual void oneStep ();

      protected:
        /// Protected constructor
        ManipulationPlanner (const core::Problem& problem,
            const core::RoadmapPtr_t& roadmap);

        /// Store weak pointer to itself
        void init (const ManipulationPlannerWkPtr_t& weak);

        /// Extend a the configuration q_near toward q_rand.
        /// \param q_near the configuration to be extended.
        /// \param q_rand the configuration toward extension is performed.
        /// \retval validPath the longest valid path (possibly of length 0),
        ///         resulting from the extension.
        /// \return True if the path needed not to be shortened to be valid.
        bool extend (const ConfigurationPtr_t &q_near,
            const ConfigurationPtr_t &q_rand, core::PathPtr_t& validPath);

        /// Extend a node of the roadmap towards a configuration.
        /// near The node of the roadmap to be extended.
        /// target The configuration towards which extension is performed.
        core::PathPtr_t extend(const core::NodePtr_t& near,
            const ConfigurationPtr_t& target,
            const graph::Edges_t& edge);

      private:
        /// Configuration shooter
        ConfigurationShooterPtr_t shooter_;
        /// The graph of constraints
        graph::GraphPtr_t constraintGraph_;
        /// weak pointer to itself
        ManipulationPlannerWkPtr_t weakPtr_;

        mutable Configuration_t qProj_;
    };
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_MANIPULATION_PLANNER_HH
