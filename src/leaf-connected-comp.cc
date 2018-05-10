// Copyright (c) 2016, Joseph Mirabel
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

#include <hpp/manipulation/leaf-connected-comp.hh>

#include <hpp/manipulation/roadmap.hh>
#include <hpp/manipulation/graph/state.hh>

namespace hpp {
  namespace manipulation {
    void LeafConnectedComp::clean (LeafConnectedComps_t& set)
    {
      for (LeafConnectedComps_t::iterator it = set.begin ();
	   it != set.end (); ++it) {
	(*it)->explored_ = false;
      }
    }

    LeafConnectedCompPtr_t LeafConnectedComp::create (const RoadmapPtr_t& roadmap)
    {
      LeafConnectedCompPtr_t shPtr =
        LeafConnectedCompPtr_t(new LeafConnectedComp(roadmap));
      shPtr->init(shPtr);
      return shPtr;
    }

    void LeafConnectedComp::addNode (const RoadmapNodePtr_t& node)
    {
      assert(state_);
      graph::StatePtr_t state = roadmap_.lock()->getState(node);

      // Sanity check
      if (state_ == state) // Oops
        throw std::runtime_error ("RoadmapNode of LeafConnectedComp must be in"
            " the same state");
      nodes_.push_back(node);
    }

    void LeafConnectedComp::setFirstNode (const RoadmapNodePtr_t& node)
    {
      assert(!state_);
      state_ = roadmap_.lock()->getState(node);
      nodes_.push_back(node);
    }

    bool LeafConnectedComp::canReach (const LeafConnectedCompPtr_t& cc)
    {
      // Store visited connected components for further cleaning.
      LeafConnectedComp::LeafConnectedComps_t explored;
      std::deque <RawPtr_t> queue;
      queue.push_back (this);
      explored_ = true;
      explored.insert (this);
      while (!queue.empty ()) {
	RawPtr_t current = queue.front ();
	queue.pop_front ();
	if (current == cc.get()) {
	  clean (explored);
	  return true;
	}
	for (LeafConnectedComp::LeafConnectedComps_t::iterator itChild =
	       current->to_.begin ();
	     itChild != current->to_.end (); ++itChild) {
	  RawPtr_t child = *itChild;
	  if (!child->explored_) {
	    child->explored_ = true;
	    explored.insert (child);
	    queue.push_back (child);
	  }
	}
      }
      clean (explored);
      return false;
    }

    bool LeafConnectedComp::canReach
    (const LeafConnectedCompPtr_t& cc,
     LeafConnectedComp::LeafConnectedComps_t& ccToThis)
    {
      bool reachable = false;
      // Store visited connected components
      LeafConnectedComp::LeafConnectedComps_t exploredForward;
      std::deque <RawPtr_t> queue;
      queue.push_back (this);
      explored_ = true;
      exploredForward.insert (this);
      while (!queue.empty ()) {
	RawPtr_t current = queue.front ();
	queue.pop_front ();
	if (current == cc.get()) {
	  reachable = true;
	  exploredForward.insert (current);
	} else {
	  for (LeafConnectedComp::LeafConnectedComps_t::iterator itChild =
		 current->to_.begin ();
	       itChild != current->to_.end (); ++itChild) {
	    RawPtr_t child = *itChild;
	    if (!child->explored_) {
	      child->explored_ = true;
	      exploredForward.insert (child);
	      queue.push_back (child);
	    }
	  }
	}
      }
      // Set visited connected components to unexplored
      clean (exploredForward);
      if (!reachable) return false;

      // Store visited connected components
      LeafConnectedComps_t exploredBackward;
      queue.push_back (cc.get());
      cc->explored_ = true;
      exploredBackward.insert (cc.get());
      while (!queue.empty ()) {
	RawPtr_t current = queue.front ();
	queue.pop_front ();
	if (current == this) {
	  exploredBackward.insert (current);
	} else {
	  for (LeafConnectedComps_t::iterator itChild =
		 current->from_.begin ();
	       itChild != current->from_.end (); ++itChild) {
	    RawPtr_t child = *itChild;
	    if (!child->explored_) {
	      child->explored_ = true;
	      exploredBackward.insert (child);
	      queue.push_back (child);
	    }
	  }
	}
      }
      // Set visited connected components to unexplored
      clean (exploredBackward);
      std::set_intersection (exploredForward.begin (), exploredForward.end (),
			     exploredBackward.begin (), exploredBackward.end (),
			     std::inserter (ccToThis, ccToThis.begin ()));
      return true;
    }

    void LeafConnectedComp::merge (const LeafConnectedCompPtr_t& other)
    {
      assert(other);
      assert(weak_.lock().get() == this);

      // Tell other's nodes that they now belong to this connected component
      for (RoadmapNodes_t::iterator itNode = other->nodes_.begin ();
	   itNode != other->nodes_.end (); ++itNode) {
	(*itNode)->leafConnectedComponent (weak_.lock ());
      }
      // Add other's nodes to this list.
      nodes_.insert (nodes_.end (), other->nodes_.begin(), other->nodes_.end());

      // Tell other's reachableTo's that other has been replaced by this
      for (LeafConnectedComps_t::iterator itcc = other->to_.begin ();
	   itcc != other->to_.end (); ++itcc) {
	(*itcc)->from_.erase (other.get());
	(*itcc)->from_.insert (this);
      }

      // Tell other's reachableFrom's that other has been replaced by this
      for (LeafConnectedComps_t::iterator itcc=other->from_.begin ();
	   itcc != other->from_.end (); ++itcc) {
	(*itcc)->to_.erase (other.get());
	(*itcc)->to_.insert (this);
      }

      LeafConnectedComps_t tmp;
      std::set_union (to_.begin (), to_.end (),
		      other->to_.begin (), other->to_.end (),
		      std::inserter (tmp, tmp.begin ()));
      to_ = tmp; tmp.clear ();
      to_.erase (other.get());
      to_.erase (this);
      std::set_union (from_.begin (), from_.end (),
		      other->from_.begin (),
		      other->from_.end (),
		      std::inserter (tmp, tmp.begin()));
      from_ = tmp; tmp.clear ();
      from_.erase (other.get());
      from_.erase (this);
    }

    WeighedLeafConnectedCompPtr_t WeighedLeafConnectedComp::create (const RoadmapPtr_t& roadmap)
    {
      WeighedLeafConnectedCompPtr_t shPtr = WeighedLeafConnectedCompPtr_t
        (new WeighedLeafConnectedComp(roadmap));
      shPtr->init(shPtr);
      return shPtr;
    }

    void WeighedLeafConnectedComp::merge (const LeafConnectedCompPtr_t& otherCC)
    {
      WeighedLeafConnectedCompPtr_t other =
        HPP_DYNAMIC_PTR_CAST(WeighedLeafConnectedComp, otherCC);
      value_type r = ((value_type)nodes_.size()) / (value_type)(nodes_.size() + other->nodes_.size());

      LeafConnectedComp::merge(otherCC);
      weight_ *= other->weight_; // TODO a geometric mean would be more natural.
      p_ = r * p_ + (1 - r) * other->p_;
    }

    void WeighedLeafConnectedComp::setFirstNode (const RoadmapNodePtr_t& node)
    {
      LeafConnectedComp::setFirstNode(node);
      std::vector<value_type> p = state_->neighbors().probabilities();
      p_.resize(p.size());
      edges_ = state_->neighbors().values();
      for (std::size_t i = 0; i < p.size(); ++i)
        p_[i] = p[i];
    }

    std::size_t WeighedLeafConnectedComp::indexOf (const graph::EdgePtr_t e) const
    {
      std::size_t i = 0;
      for (; i < edges_.size(); ++i)
        if (edges_[i] == e) break;
      return i;
    }
  } //   namespace manipulation
} // namespace hpp
