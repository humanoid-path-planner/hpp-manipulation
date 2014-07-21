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
        wkPtr_ = weak;
      }

      NodePtr_t NodeSelector::createNode (const ConstraintPtr_t& constraints)
      {
        NodePtr_t newNode = Node::create(constraints);
        orderedStates_.push_back(newNode);
        return newNode;
      }
    } // namespace graph
  } // namespace manipulation
} // namespace hpp
