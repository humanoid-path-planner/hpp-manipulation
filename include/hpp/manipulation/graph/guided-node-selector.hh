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

#ifndef HPP_MANIPULATION_GRAPH_GUIDED_NODE_SELECTOR_HH
# define HPP_MANIPULATION_GRAPH_GUIDED_NODE_SELECTOR_HH

#include "hpp/manipulation/graph/guided-state-selector.hh"

namespace hpp {
  namespace manipulation {
    namespace graph {
      typedef GuidedStateSelector GuidedNodeSelector
      HPP_MANIPULATION_DEPRECATED;
      typedef boost::shared_ptr < GuidedNodeSelector > GuidedNodeSelectorPtr_t;
    } // namespace graph
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_GRAPH_GUIDED_NODE_SELECTOR_HH
