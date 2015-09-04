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

#include "hpp/manipulation/graph/guided-node-selector.hh"

#include <hpp/util/assertion.hh>
#include <hpp/model/configuration.hh>
#include <hpp/core/steering-method.hh>

#include "../astar.hh"
#include "hpp/manipulation/roadmap.hh"
#include "hpp/manipulation/roadmap-node.hh"

#include <cstdlib>

namespace hpp {
  namespace manipulation {
    namespace graph {
      GuidedNodeSelectorPtr_t GuidedNodeSelector::create(const std::string& name,
          const core::RoadmapPtr_t& roadmap)
      {
        GuidedNodeSelector* ptr = new GuidedNodeSelector (name, roadmap);
        GuidedNodeSelectorPtr_t shPtr (ptr);
        ptr->init (shPtr);
        return shPtr;
      }

      void GuidedNodeSelector::init (const GuidedNodeSelectorPtr_t& weak)
      {
        NodeSelector::init (weak);
        wkPtr_ = weak;
      }

      void GuidedNodeSelector::setNodeList (const Nodes_t& nodeList)
      {
        nodeList_ = nodeList;
      }

      EdgePtr_t GuidedNodeSelector::chooseEdge(RoadmapNodePtr_t from) const
      {
        if (nodeList_.empty ()) return NodeSelector::chooseEdge (from);
        Astar::Nodes_t list;
        bool reverse = false;
        if (from->connectedComponent () == roadmap_->initNode ()->connectedComponent ()) {
          Astar alg (roadmap_->distance (), wkPtr_.lock(), static_cast <RoadmapNodePtr_t> (roadmap_->initNode ()));
          list = alg.solution (from);
        } else {
          core::Nodes_t::const_iterator itg = roadmap_->goalNodes ().begin ();
          for (; itg != roadmap_->goalNodes ().end (); ++itg)
            if ((*itg)->connectedComponent () == from->connectedComponent ())
              break;
          if (itg == roadmap_->goalNodes ().end ()) {
            hppDout (error, "This configuration can reach neither the initial "
                "configuration nor any of the goal configurations.");
            return EdgePtr_t ();
          }
          reverse = true;
          Astar alg (roadmap_->distance (), wkPtr_.lock(), from);
          list = alg.solution (static_cast <RoadmapNodePtr_t> (*itg));
        }
        std::unique (list.begin(), list.end ());
        // Check if the beginning of nodeList is list
        if (list.size() <= nodeList_.size()) {
          Neighbors_t nn;
          if (reverse) {
            Nodes_t::const_reverse_iterator it1 = nodeList_.rbegin ();
            Astar::Nodes_t::const_reverse_iterator it2 = list.rbegin();
            Astar::Nodes_t::const_reverse_iterator itEnd2 = list.rend();
            do {
              if (*it1 != *it2) {
                hppDout (error, "The target sequence of nodes does not end with "
                    "the sequence of nodes to reach this configuration.");
                return EdgePtr_t ();
              }
              ++it1;
            } while (++it2 != itEnd2);
            NodePtr_t node = getNode (from);
            HPP_ASSERT (node == list.front ());
            const Neighbors_t& n = node->neighbors();
            /// You stay in the same node
            for (Neighbors_t::const_iterator it = n.begin (); it != n.end (); ++it)
              if (it->second->to () == node)
                nn.insert (it->second, it->first);
            const Neighbors_t& n1 = (*it1)->neighbors ();
            /// Go from node it1 to node
            for (Neighbors_t::const_iterator it = n1.begin (); it != n1.end (); ++it)
              if (it->second->to () == node)
                nn.insert (it->second, it->first);
          } else {
            Nodes_t::const_iterator it1 = nodeList_.begin ();
            Astar::Nodes_t::const_iterator it2 = list.begin();
            Astar::Nodes_t::const_iterator itEnd2 = list.end();
            do {
              if (*it1 != *it2) {
                hppDout (error, "The target sequence of nodes does not start with "
                    "the sequence of nodes to reach this configuration.");
                return EdgePtr_t ();
              }
              ++it1;
            } while (++it2 != itEnd2);
            NodePtr_t node = getNode (from);
            HPP_ASSERT (node == list.back ());
            const Neighbors_t& n = node->neighbors();
            for (Neighbors_t::const_iterator it = n.begin (); it != n.end (); ++it)
              /// You stay in the same node
              /// or go from node to node it1 
              if (it->second->to () == node || it->second->to () == *it1)
                nn.insert (it->second, it->first);
          }
          if (nn.size () > 0)
            return nn ();
          hppDout (error, "This node has no neighbors to get to an admissible states.");
        }
        return EdgePtr_t ();
      }

      std::ostream& GuidedNodeSelector::dotPrint (std::ostream& os, dot::DrawingAttributes) const
      {
        for (Nodes_t::const_iterator it = orderedStates_.begin();
            orderedStates_.end() != it; ++it)
          (*it)->dotPrint (os);
        return os;
      }

      std::ostream& GuidedNodeSelector::print (std::ostream& os) const
      {
        os << "|-- ";
        GraphComponent::print (os) << std::endl;
        for (Nodes_t::const_iterator it = orderedStates_.begin();
            orderedStates_.end() != it; ++it)
          os << *(*it);
        return os;
      }
    } // namespace graph
  } // namespace manipulation
} // namespace hpp
