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

#ifndef HPP_MANIPULATION_GRAPHGRAPH_HH
# define HPP_MANIPULATION_GRAPHGRAPH_HH

#include "hpp/manipulation/fwd.hh"
#include "hpp/manipulation/graph/fwd.hh"

namespace hpp {
  namespace manipulation {
    namespace graph {
      /// Description of the constraint graph
      /// This class contains a graph representing a robot with several
      /// end-effectors.
      class HPP_MANIPULATION_DLLAPI Graph
      {
        public:
          /// Returns the states of a configuration.
          virtual Nodes_t getNode(const Configuration_t config) const;

          /// Select randomly outgoing edges of the given nodes.
          virtual Edges_t chooseEdge(const Nodes_t& node) const;

        private:
          /// This list contains a node selector for each end-effector.
          set::list < NodeSelectorPtr_t > nodeSelectors_;

          /// A set of constraints that will always be used, for example
          /// stability constraints.
          ConstraintPtr_t constraints_;
      }; // Class Graph
    } // namespace graph
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_GRAPHGRAPH_HH
