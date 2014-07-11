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

#ifndef HPP_MANIPULATION_GRAPH_NODE_SELECTOR_HH
# define HPP_MANIPULATION_GRAPH_NODE_SELECTOR_HH

#include "hpp/manipulation/fwd.hh"
#include "hpp/manipulation/graph/gripper-state.hh"

namespace hpp {
  namespace manipulation {
    namespace graph {
      /// This class is used to get the state of a configuration. States have to
      /// be ordered because a configuration can be in several states.
      class HPP_MANIPULATION_DLLAPI NodeSelector
      {
        public:
          /// Create a nodes with the constraints
          NodePtr_t createNode (const ConstraintPtr_t& constraints)
          {
            NodePtr_t newNode = Node::create(constraints);
            orderedStates_.push_back(newNode);
            return newNode;
          }

          /// Returns the state of a configuration.
          virtual NodePtr_t getNode(const Configuration_t config) const;

          /// Select randomly an outgoing edge of the given node.
          virtual EdgePtr_t chooseEdge(const NodePtr_t& node) const;

        private:
          /// List of the states of one end-effector, ordered by priority.
          std::vector< NodePtr_t > orderedStates_;
      }; // Class NodeSelector
    } // namespace graph
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_GRAPH_NODE_SELECTOR_HH
