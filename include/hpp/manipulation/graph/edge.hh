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

#ifndef HPP_MANIPULATION_GRAPH_EDGE_HH
# define HPP_MANIPULATION_GRAPH_EDGE_HH

#include <hpp/core/constraints-set.hh>

#include "hpp/manipulation/fwd.hh"
#include "hpp/manipulation/graph/gripper-state.hh"

namespace hpp {
  namespace manipulation {
    namespace graph {
      /// Transition between states of a end-effector.
      ///
      /// Vertices of the graph of constraints.
      class HPP_MANIPULATION_DLLAPI Edge
      {
        public:
          /// Projector to project onto the same leaf as config.
          /// \return The initialized projector.
          /// \param config Configuration that will initialize the projector.
          ConfigProjectorPtr_t configurationProjector(const Configuration_t config);

          /// Projector to project a path.
          /// \return The initialized projector.
          /// \param config Configuration that will initialize the projector.
          ConfigProjectorPtr_t pathProjector(const Configuration_t config);

        private:
          /// The two ends of the transition.
          NodeWkPtr_t startState, endState;

          /// Set of constraints to be statisfied.
          ConstraintSetPtr_t constraints_;

          /// Projectors ensuring that a path between q_near and q_proj is
          /// valid regarding the manipulation rules.
          ConfigProjectorPtr_t configurationProjector_;

          /// Projectors ensuring that a path between two configurations in
          /// the same leaf lies in the leaf.
          ConfigProjectorPtr_t pathProjector_;
      }; // class Edge
    } // namespace graph
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_GRAPH_EDGE_HH
