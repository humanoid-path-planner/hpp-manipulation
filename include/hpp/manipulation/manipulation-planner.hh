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
#include <hpp/core/roadmap.hh>

#include <hpp/statistics/success-bin.hh>

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

        /// One step of extension.
        /// 
        /// A set of constraints is choosen using the graph of constraints.
        /// A constraint extension is done using a choosen set.
        ///
        virtual void oneStep ();

        /// Extend a the configuration q_near toward q_rand.
        /// \param q_near the configuration to be extended.
        /// \param q_rand the configuration toward extension is performed.
        /// \retval validPath the longest valid path (possibly of length 0),
        ///         resulting from the extension.
        /// \return True if the returned path is valid.
        bool extend (const ConfigurationPtr_t &q_near,
            const ConfigurationPtr_t &q_rand, core::PathPtr_t& validPath);

      protected:
        /// Protected constructor
        ManipulationPlanner (const Problem& problem,
            const core::RoadmapPtr_t& roadmap);

        /// Store weak pointer to itself
        void init (const ManipulationPlannerWkPtr_t& weak);

      private:
        /// Try to connect configurations in a list.
        void tryConnect (const core::Nodes_t nodes);

        /// Configuration shooter
        ConfigurationShooterPtr_t shooter_;
        /// Pointer to the problem
        const Problem& problem_;
        /// weak pointer to itself
        ManipulationPlannerWkPtr_t weakPtr_;

        /// Keep track of success and failure for method
        /// extend.
        typedef ::hpp::statistics::SuccessStatistics SuccessStatistics;
        typedef ::hpp::statistics::SuccessBin SuccessBin;
        SuccessStatistics extendStatistics_;

        /// A Reason is associated to each Edges_t that generated a failure.
        enum TypeOfFailure {
          PROJECTION,
          STEERING_METHOD,
          PATH_VALIDATION
        };
        struct Reasons {
          typedef ::hpp::statistics::SuccessBin::Reason Reason;
          Reason projFailed, smFailed, pvFailed;

          Reasons (const Reason& proj, const Reason& sm, const Reason& pv) :
            projFailed (proj), smFailed (sm), pvFailed (pv) {}
          const Reason& get (TypeOfFailure t)
          {
            switch (t) {
              case PROJECTION:
                return projFailed;
              case STEERING_METHOD:
                return smFailed;
              case PATH_VALIDATION:
                return pvFailed;
            }
            return ::hpp::statistics::SuccessBin::REASON_UNKNOWN;
          }
        };
        typedef std::pair < graph::Edges_t, Reasons > EdgesReasonPair;
        typedef std::map  < graph::Edges_t, Reasons > EdgesReasonMap;
        EdgesReasonMap failureReasons_;

        void addFailure (TypeOfFailure t, const graph::Edges_t& edges);

        mutable Configuration_t qProj_;
    };
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_MANIPULATION_PLANNER_HH
