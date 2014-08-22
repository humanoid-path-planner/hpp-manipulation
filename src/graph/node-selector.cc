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

#include <cstdlib>

#include "hpp/manipulation/graph/node-selector.hh"

namespace hpp {
  namespace manipulation {
    namespace graph {
      NodeSelectorPtr_t NodeSelector::create()
      {
        NodeSelector* ptr = new NodeSelector();
        NodeSelectorPtr_t shPtr (ptr);
        ptr->init (shPtr);
        return shPtr;
      }

      void NodeSelector::init (const NodeSelectorPtr_t& weak)
      {
        GraphComponent::init (weak);
        wkPtr_ = weak;
      }

      NodePtr_t NodeSelector::createNode ()
      {
        NodePtr_t newNode = Node::create();
        newNode->nodeSelector(wkPtr_);
        newNode->parentGraph(graph_);
        orderedStates_.push_back(newNode);
        return newNode;
      }

      NodePtr_t NodeSelector::getNode(const Configuration_t config) const
      {
        for (Nodes_t::const_iterator it = orderedStates_.begin();
            orderedStates_.end() != it; it++)
          if ((*it)->contains(config))
            return *it;
        throw std::logic_error ("A configuration has no node");
        return NodePtr_t ();
      }

      EdgePtr_t NodeSelector::chooseEdge(const NodePtr_t& node) const
      {
        const Edges_t neighbors = node->neighbors();
        size_t n = rand() % neighbors.size();
        return neighbors[n];
      }

      std::ostream& NodeSelector::print (std::ostream& os) const
      {
        GraphComponent::print (os << "|-- ") << std::endl;
        for (Nodes_t::const_iterator it = orderedStates_.begin();
            orderedStates_.end() != it; it++)
          os << *(*it);
        return os;
      }
    } // namespace graph
  } // namespace manipulation
} // namespace hpp
