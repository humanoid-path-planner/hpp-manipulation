//
// Copyright (c) 2015 CNRS
// Authors: Florent Lamiraux, Joseph Mirabel
//
// This file is part of hpp-manipulation.gc
// hpp-manipulation.gc is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-manipulation.gc is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-manipulation.gc  If not, see
// <http://www.gnu.org/licenses/>.

#ifndef HPP_MANIPULATION_ASTAR_HH
# define HPP_MANIPULATION_ASTAR_HH

# include <limits>
# include <hpp/core/distance.hh>
# include <hpp/core/node.hh>
# include <hpp/core/edge.hh>

# include <hpp/manipulation/fwd.hh>
# include <hpp/manipulation/roadmap-node.hh>
# include <hpp/manipulation/graph/node-selector.hh>
//# include <hpp/core/path-vector.hh>

namespace hpp {
  namespace manipulation {
    class Astar
    {
    public:
      typedef std::map <RoadmapNodePtr_t, value_type> CostMap_t;
      struct CostMapCompFunctor {
	CostMap_t& cost_;
	CostMapCompFunctor (CostMap_t& cost) : cost_ (cost) {}
	bool operator () (const RoadmapNodePtr_t& n1, const RoadmapNodePtr_t& n2)
	{ return cost_ [n1] < cost_ [n2]; }
	bool operator () (const RoadmapNodePtr_t& n1, const value_type& val)
	{ return cost_ [n1] < val; }
      }; // struc CostMapCompFunctor

      typedef std::list <graph::NodePtr_t> Nodes_t;
      typedef std::list <RoadmapNodePtr_t> RoadmapNodes_t;
      typedef std::list <core::EdgePtr_t> RoadmapEdges_t;
      typedef std::map <RoadmapNodePtr_t, core::EdgePtr_t> Parent_t;

      Astar (const core::DistancePtr_t distance,
          const graph::NodeSelectorPtr_t& nodeSelector, RoadmapNodePtr_t from) :
	distance_ (distance), selector_ (nodeSelector),
        from_ (from)
      {
        open_.push_back (from);
        costFromStart_ [from] = 0;
      }

      Nodes_t solution (RoadmapNodePtr_t to)
      {
	if (parent_.find (to) != parent_.end () ||
            findPath (to))
        {
          RoadmapNodePtr_t node = to;
          Nodes_t nodes;

          while (node) {
            Parent_t::const_iterator itNode = parent_.find (node);
            if (itNode != parent_.end ()) {
              node = static_cast <RoadmapNodePtr_t> (itNode->second->from ());
              nodes.push_front (selector_->getNode (node));
            }
            else node = RoadmapNodePtr_t (0);
          }
          // We may want to clean it a little
          // std::unique (nodes.begin(), nodes.end ());
          return nodes;
        }
        return Nodes_t();
      }

    private:
      bool findPath (RoadmapNodePtr_t to)
      {
        // Recompute the estimated cost to goal
        for (CostMap_t::iterator it = estimatedCostToGoal_.begin ();
            it != estimatedCostToGoal_.end (); ++it) {
          it->second = getCostFromStart (it->first) + heuristic (it->first, to);
        }
        open_.sort (CostMapCompFunctor (estimatedCostToGoal_));

	while (!open_.empty ()) {
	  RoadmapNodes_t::iterator itv = open_.begin ();
          RoadmapNodePtr_t current (*itv);
	  if (current == to) {
	    return true;
	  }
	  open_.erase (itv);
	  closed_.push_back (current);
	  for (RoadmapEdges_t::const_iterator itEdge = current->outEdges ().begin ();
	       itEdge != current->outEdges ().end (); ++itEdge) {
            RoadmapNodePtr_t child = static_cast <RoadmapNodePtr_t> ((*itEdge)->to ());
	    if (std::find (closed_.begin(), closed_.end(),
                  child) == closed_.end ()) {
	      // node is not in closed set
              value_type transitionCost = edgeCost (*itEdge);
              value_type tmpCost = getCostFromStart (current) + transitionCost;
	      bool childNotInOpenSet = (std::find (open_.begin (),
						   open_.end (),
						   child) == open_.end ());
	      if ((childNotInOpenSet) || (tmpCost < getCostFromStart (child))) {
		parent_ [child] = *itEdge;
		costFromStart_ [child] = tmpCost;
                value_type estimatedCost = tmpCost + heuristic (child, to);
                estimatedCostToGoal_ [child] = estimatedCost;
                if (childNotInOpenSet) {
                  // Find the first element not strictly smaller than child
                  RoadmapNodes_t::iterator pos =
                    std::lower_bound (open_.begin (), open_.end (),
                        estimatedCost, CostMapCompFunctor (estimatedCostToGoal_));
                  open_.insert (pos, child);
                }
	      }
	    }
	  }
	}
        return false;
      }

      inline value_type heuristic (RoadmapNodePtr_t node, RoadmapNodePtr_t to) const
      {
	const ConfigurationPtr_t& config = node->configuration ();
	return (*distance_) (*config, *to->configuration ());
      }

      inline value_type edgeCost (const core::EdgePtr_t& edge) const
      {
	return edge->path ()->length ();
      }

      value_type getCostFromStart (RoadmapNodePtr_t to) const
      {
        CostMap_t::const_iterator it = costFromStart_.find (to);
        if (it == costFromStart_.end())
          return std::numeric_limits <value_type>::max();
        return it->second;
      }

      RoadmapNodes_t closed_;
      RoadmapNodes_t open_;
      std::map <RoadmapNodePtr_t, value_type> costFromStart_;
      std::map <RoadmapNodePtr_t, value_type> estimatedCostToGoal_;
      Parent_t parent_;
      core::DistancePtr_t distance_;
      graph::NodeSelectorPtr_t selector_;
      RoadmapNodePtr_t from_;

    }; // class Astar
  } // namespace manipulation
} // namespace hpp


#endif // HPP_MANIPULATION_ASTAR_HH
