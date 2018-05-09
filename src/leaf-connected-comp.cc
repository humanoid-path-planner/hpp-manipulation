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

    bool LeafConnectedComp::canMerge (const LeafConnectedCompPtr_t& otherCC) const
    {
      if (otherCC->state_ != state_) return false;
      LeafConnectedComps_t::const_iterator it = std::find
        (to_.begin(), to_.end(), otherCC.get());
      if (it == to_.end()) return false;
      it = std::find
        (from_.begin(), from_.end(), otherCC.get());
      if (it == from_.end()) return false;
      return true;
    }

    void LeafConnectedComp::canReach (const LeafConnectedCompPtr_t& otherCC)
    {
      to_.insert(otherCC.get());
      otherCC->from_.insert(this);
    }

    void LeafConnectedComp::merge (LeafConnectedCompPtr_t other)
    {
      assert (canMerge(other));
      if (other == weak_.lock()) return;

      // Tell other's nodes that they now belong to this connected component
      for (RoadmapNodes_t::iterator itNode = other->nodes_.begin ();
	   itNode != other->nodes_.end (); ++itNode) {
	(*itNode)->symbolicComponent (weak_.lock ());
      }
      // Add other's nodes to this list.
      nodes_.insert (nodes_.end (), other->nodes_.begin(), other->nodes_.end());

      from_.erase (other.get());
      other->from_.erase (this);
      from_.insert (other->from_.begin(), other->from_.end());
      to_.erase (other.get());
      other->to_.erase (this);
      to_.insert (other->to_.begin(), other->to_.end());
    }

    WeighedLeafConnectedCompPtr_t WeighedLeafConnectedComp::create (const RoadmapPtr_t& roadmap)
    {
      WeighedLeafConnectedCompPtr_t shPtr = WeighedLeafConnectedCompPtr_t
        (new WeighedLeafConnectedComp(roadmap));
      shPtr->init(shPtr);
      return shPtr;
    }

    void WeighedLeafConnectedComp::merge (LeafConnectedCompPtr_t otherCC)
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
