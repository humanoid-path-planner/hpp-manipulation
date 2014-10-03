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

#include "hpp/manipulation/robot.hh"
#include "hpp/manipulation/graph/edge.hh"
#include "hpp/manipulation/graph/graph.hh"
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

      void Node::init (const NodeWkPtr_t& weak)
      {
        GraphComponent::init (weak);
        wkPtr_ = weak;
      }

      EdgePtr_t Node::linkTo(const NodePtr_t& to, const Weight_t& w, const bool& isInNodeFrom)
      {
        EdgePtr_t newEdge = Edge::create(wkPtr_, to);
        neighbors_.insert (newEdge, w);
        newEdge->parentGraph(graph_);
        newEdge->isInNodeFrom (isInNodeFrom);
        return newEdge;
      }

      bool Node::contains (const Configuration_t config)
      {
        return configConstraint()->isSatisfied (config);
      }

      std::ostream& Node::print (std::ostream& os) const
      {
        os << "|   |-- " << (GraphComponent*)this << std::endl;
        for (Neighbors_t::const_iterator it = neighbors_.begin();
            it != neighbors_.end(); it++)
          os << *(it->second);
        return os;
      }

      ConstraintPtr_t Node::configConstraint()
      {
        if (!configConstraints_) {
          configConstraints_ = graph_.lock ()->configConstraint (wkPtr_.lock ());
        }
        return configConstraints_;
      }
    } // namespace graph
  } // namespace manipulation
} // namespace hpp
