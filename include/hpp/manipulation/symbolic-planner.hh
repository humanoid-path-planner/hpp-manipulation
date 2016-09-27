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

#ifndef HPP_MANIPULATION_SYMBOLIC_PLANNER_HH
# define HPP_MANIPULATION_SYMBOLIC_PLANNER_HH

#include <hpp/core/path-planner.hh>

#include <hpp/statistics/success-bin.hh>

#include "hpp/manipulation/graph/statistics.hh"

#include "hpp/manipulation/fwd.hh"
#include "hpp/manipulation/config.hh"
#include "hpp/manipulation/graph/fwd.hh"

namespace hpp {
  namespace manipulation {
    /// \addtogroup path_planning
    /// \{

    class HPP_MANIPULATION_DLLAPI SymbolicPlanner :
      public hpp::core::PathPlanner
    {
      public:
        typedef std::list<std::size_t> ErrorFreqs_t;
        struct ExtendStatus {
          int status;
          int info;
          graph::EdgePtr_t edge;
          PathValidationReportPtr_t validationReport;
        };

        /// Create an instance and return a shared pointer to the instance
        static SymbolicPlannerPtr_t create (const core::Problem& problem,
            const core::RoadmapPtr_t& roadmap);

        /// One step of extension.
        ///
        /// A set of constraints is chosen using the graph of constraints.
        /// A constraint extension is done using a chosen set.
        ///
        virtual void oneStep ();

        /// Extend configuration q_near toward q_rand.
        /// \param q_near the configuration to be extended.
        /// \param q_rand the configuration toward extension is performed.
        /// \retval validPath the longest valid path (possibly of length 0),
        ///         resulting from the extension.
        /// \retval status an integer
        /// \return True if the returned path is valid.
        bool extend (RoadmapNodePtr_t q_near, const ConfigurationPtr_t &q_rand,
            core::PathPtr_t& validPath, ExtendStatus& status);

        /// Get the number of occurrence of each errors.
        ///
        /// \sa ManipulationPlanner::errorList
        ErrorFreqs_t getEdgeStat (const graph::EdgePtr_t& edge) const;

        /// Get the list of possible outputs of the extension step.
        ///
        /// \sa ManipulationPlanner::getEdgeStat
        static StringList_t errorList ();

      protected:
        /// Protected constructor
        SymbolicPlanner (const Problem& problem,
            const RoadmapPtr_t& roadmap);

        /// Store weak pointer to itself
        void init (const SymbolicPlannerWkPtr_t& weak);

      private:
        /// Try to connect nodes of the roadmap to other connected components.
        /// \return the number of connection made.
        std::size_t tryConnectToRoadmap (const core::Nodes_t nodes);
        /// Try to connect nodes in a list between themselves.
        /// \return the number of connection made.
        std::size_t tryConnectNewNodes (const core::Nodes_t nodes);

        void updateWeightsAndProbabilities (
            const RoadmapNodePtr_t& near, const RoadmapNodePtr_t& newN,
            const ExtendStatus& es);

        /// The transitions probabilities are updated as follow:
        ///
        /// \li let \f$p_{edge}\f$ be the edge probability before update,
        /// \li the edge probability is multiplied by \f$\alpha\f$,
        /// \li all other probabilities are multiplied by \f$ \frac{1 - \alpha * p_{edge}}{1 - p_{edge}} \f$
        ///
        /// \param alpha should be between 0 and 1.
        ///
        /// \note Theoretically, the described operation preserves the
        /// normalization of the sum. WeighedSymbolicComponent::normalizeProba
        /// is called to avoid numerical errors.
        static void updateEdgeProba (
            const graph::EdgePtr_t edge,
            WeighedSymbolicComponentPtr_t wsc,
            const value_type alpha);

        /// Configuration shooter
        ConfigurationShooterPtr_t shooter_;
        /// Pointer to the problem
        const Problem& problem_;
        /// Pointer to the roadmap
        RoadmapPtr_t roadmap_;
        /// weak pointer to itself
        SymbolicPlannerWkPtr_t weakPtr_;

        /// Keep track of success and failure for method
        /// extend.
        typedef ::hpp::statistics::SuccessStatistics SuccessStatistics;
        typedef ::hpp::statistics::SuccessBin SuccessBin;
        typedef ::hpp::statistics::SuccessBin::Reason Reason;
        SuccessStatistics& edgeStat (const graph::EdgePtr_t& edge);
        std::vector<size_type> indexPerEdgeStatistics_;
        std::vector<SuccessStatistics> perEdgeStatistics_;

        /// A Reason is associated to each EdgePtr_t that generated a failure.
        enum TypeOfFailure {
          SUCCESS = -1,
          PROJECTION = 0,
          STEERING_METHOD = 1,
          PATH_VALIDATION_ZERO = 2,
          PATH_PROJECTION_ZERO = 3,
          UNKNOWN = 4,
          PATH_PROJECTION_SHORTER = 5,
          PATH_VALIDATION_SHORTER = 6,
          PARTLY_EXTENDED = 7,
          PATH_PROJECTION_AND_VALIDATION_SHORTER = PATH_PROJECTION_SHORTER + PATH_VALIDATION_SHORTER
        };
        static const std::vector<Reason> reasons_;

        const value_type extendStep_;

        mutable Configuration_t qProj_;
    };
    /// \}
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_SYMBOLIC_PLANNER_HH
