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

#include "hpp/manipulation/graph/node.hh"

namespace hpp {
  namespace manipulation {
    namespace graph {
      NodePtr_t Node::create ()
      {
        Node* node = new Node;
        NodePtr_t shPtr(node);
        shPtr->init(shPtr);
        return shPtr;
      }

      NodePtr_t Node::create (const ConstraintPtr_t& constraints)
      {
        Node* node = new Node;
        NodePtr_t shPtr(node);
        shPtr->init(shPtr, constraints);
        return shPtr;
      }

      void Node::init (const NodeWkPtr_t& weak)
      {
        GraphComponent::init (weak);
        wkPtr_ = weak;
      }

      void Node::init (const NodeWkPtr_t& self, const ConstraintPtr_t& constraint)
      {
        init(self);
        constraints(constraint);
      }

      EdgePtr_t Node::linkTo(const NodePtr_t& to, const ConstraintPtr_t& constraints)
      {
        EdgePtr_t newEdge = Edge::create(wkPtr_, to, constraints);
        neighbors_.push_back(newEdge);
        return newEdge;
      }

      bool Node::contains (const Configuration_t config) const
      {
        // TODO: This is not the most efficient way. We should
        // compute the value of the constraint instead of apllying
        // the constraint.
        Configuration_t cfg = config;
        return constraints_->apply(cfg) && ( cfg == config );
      }

      std::ostream& Node::print (std::ostream& os) const
      {
        os << " |    |_ " << name() << std::endl;
        for (Edges_t::const_iterator it = neighbors_.begin();
            it != neighbors_.end(); it++)
          os << *(*it);
        return os;
      }
    } // namespace graph
  } // namespace manipulation
} // namespace hpp
