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

#ifndef HPP_MANIPULATION_GRAPH_NODE_HH
# define HPP_MANIPULATION_GRAPH_NODE_HH

#include <hpp/core/constraint-set.hh>

#include "hpp/manipulation/fwd.hh"
#include "hpp/manipulation/graph/fwd.hh"
#include "hpp/manipulation/graph/edge.hh"

namespace hpp {
  namespace manipulation {
    namespace graph {
      /// State of an end-effector.
      ///
      /// Nodes of the graph of constraints. There is one
      /// graph for each end-effector.
      class HPP_MANIPULATION_DLLAPI Node
      {
        public:
          /// Create a new node.
          static NodePtr_t create (const ConstraintPtr_t& constraints)
          {
            Node* node = new Node;
            NodePtr_t shPtr(node);
            shPtr->init(shPtr, constraints);
            return shPtr;
          }

          /// Create a link from this node to the given node.
          EdgePtr_t linkTo(const NodePtr_t& to, const ConstraintPtr_t& constraints)
          {
            EdgePtr_t newEdge = Edge::create(wkPtr_, to, constraints);
            neighbors_.push_back(newEdge);
            return newEdge;
          }

          /// Check whether the configuration is in this state.
          /// \return True if this state contains this configuration
          /// \param config The configuration to be tested.
          /// \note You should note use that to know in which states a
          /// configuration is. This only checks if the configuration satisfies
          /// the constraints. Instead, use the class NodeSelector.
          virtual bool contains (const Configuration_t config) const
          {
            // TODO: This is not the most efficient way. We should
            // compute the value of the constraint instead of apllying
            // the constraint.
            Configuration_t cfg = config;
            return constraints_->apply(cfg) && ( cfg == config );
          }

          /// Get the constraint set associated to the node.
          const ConstraintPtr_t constraints () const
          {
            return constraints_;
          }

          /// Set the constraint set associated to the node.
          void constraints (const ConstraintPtr_t& constraints)
          {
            constraints_ = constraints;
          }

        protected:
          /// Initialize the object.
          void init (const NodeWkPtr_t& self, const ConstraintPtr_t& constraints)
          {
            wkPtr_ = self;
            constraints_ = constraints;
          }

          Node()
          {}

        private:
          /// List of possible motions from this state (i.e. the outgoing
          /// vertices).
          std::vector < EdgePtr_t > neighbors_;

          /// Set of constraints to be statisfied.
          ConstraintPtr_t constraints_;

          /// A selector that will implement the selection of the next state.
          NodeSelectorPtr_t selector_;

          /// Weak pointer to itself.
          NodeWkPtr_t wkPtr_;
      }; // class Node
    } // namespace graph
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_GRAPH_NODE_HH
