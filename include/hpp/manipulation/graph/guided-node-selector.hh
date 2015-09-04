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

#include "hpp/manipulation/fwd.hh"
#include "hpp/manipulation/graph/fwd.hh"
#include "hpp/manipulation/graph/node-selector.hh"

namespace hpp {
  namespace manipulation {
    namespace graph {
      class HPP_MANIPULATION_DLLAPI GuidedNodeSelector : public NodeSelector
      {
        public:
          /// Create a new GuidedNodeSelector.
          static GuidedNodeSelectorPtr_t create(const std::string& name,
              const core::RoadmapPtr_t& roadmap);

          /// Set the target
          void setNodeList (const Nodes_t& nodeList);

          /// Select randomly an outgoing edge of the given node.
          virtual EdgePtr_t chooseEdge(RoadmapNodePtr_t from) const;

          /// Print the object in a stream.
          std::ostream& dotPrint (std::ostream& os, dot::DrawingAttributes da = dot::DrawingAttributes ()) const;

        protected:
          /// Initialization of the object.
          void init (const GuidedNodeSelectorPtr_t& weak);

          /// Constructor
          GuidedNodeSelector (const std::string& name,
              const core::RoadmapPtr_t roadmap) :
            NodeSelector (name), roadmap_ (roadmap)
          {}

          /// Print the object in a stream.
          std::ostream& print (std::ostream& os) const;

        private:
          /// The target
          Nodes_t nodeList_;

          /// The roadmap
          core::RoadmapPtr_t roadmap_;

          /// Weak pointer to itself.
          GuidedNodeSelectorWkPtr_t wkPtr_;
      }; // Class NodeSelector
    } // namespace graph
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_GRAPH_GUIDED_NODE_SELECTOR_HH
