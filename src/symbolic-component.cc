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

#include <hpp/manipulation/symbolic-component.hh>

#include <hpp/manipulation/roadmap.hh>
#include <hpp/manipulation/graph/state.hh>

namespace hpp {
  namespace manipulation {
    SymbolicComponentPtr_t SymbolicComponent::create (const RoadmapPtr_t& roadmap)
    {
      SymbolicComponentPtr_t shPtr =
        SymbolicComponentPtr_t(new SymbolicComponent(roadmap));
      shPtr->init(shPtr);
      return shPtr;
    }

    void SymbolicComponent::addNode (const RoadmapNodePtr_t& node)
    {
      assert(state_);
      graph::StatePtr_t state = roadmap_->getState(node);

      // Sanity check
      if (state_ == state) // Oops
        throw std::runtime_error ("RoadmapNode of SymbolicComponent must be in"
            " the same state");
      nodes_.push_back(node);
    }

    void SymbolicComponent::setFirstNode (const RoadmapNodePtr_t& node)
    {
      assert(!state_);
      state_ = roadmap_->getState(node);
      nodes_.push_back(node);
    }

    bool SymbolicComponent::canMerge (const SymbolicComponentPtr_t& otherCC) const
    {
      if (otherCC->state_ != state_) return false;
      SymbolicComponents_t::const_iterator it = std::find
        (to_.begin(), to_.end(), otherCC);
      if (it == to_.end()) return false;
      it = std::find
        (from_.begin(), from_.end(), otherCC);
      if (it == from_.end()) return false;
      return true;
    }

    void SymbolicComponent::canReach (const SymbolicComponentPtr_t& otherCC)
    {
      to_.insert(otherCC);
      otherCC->from_.insert(weak_.lock());
    }

    void SymbolicComponent::merge (SymbolicComponentPtr_t other)
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

      from_.erase (other);
      other->from_.erase (weak_.lock());
      from_.insert (other->from_.begin(), other->from_.end());
      to_.erase (other);
      other->to_.erase (weak_.lock());
      to_.insert (other->to_.begin(), other->to_.end());
    }

    WeighedSymbolicComponentPtr_t WeighedSymbolicComponent::create (const RoadmapPtr_t& roadmap)
    {
      WeighedSymbolicComponentPtr_t shPtr = WeighedSymbolicComponentPtr_t
        (new WeighedSymbolicComponent(roadmap));
      shPtr->init(shPtr);
      return shPtr;
    }

    void WeighedSymbolicComponent::merge (SymbolicComponentPtr_t otherCC)
    {
      WeighedSymbolicComponentPtr_t other =
        HPP_DYNAMIC_PTR_CAST(WeighedSymbolicComponent, otherCC);
      value_type r = nodes_.size() / (nodes_.size() + other->nodes_.size());

      SymbolicComponent::merge(otherCC);
      weight_ *= other->weight_; // TODO a geometric mean would be more natural.
      p_ = r * p_ + (1 - r) * other->p_;
    }

    void WeighedSymbolicComponent::setFirstNode (const RoadmapNodePtr_t& node)
    {
      SymbolicComponent::setFirstNode(node);
      std::vector<value_type> p = state_->neighbors().probabilities();
      p_.resize(p.size());
      edges_ = state_->neighbors().values();
      for (std::size_t i = 0; i < p.size(); ++i)
        p_[i] = p[i];
    }

    std::size_t WeighedSymbolicComponent::indexOf (const graph::EdgePtr_t e) const
    {
      std::size_t i = 0;
      for (; i < edges_.size(); ++i)
        if (edges_[i] == e) break;
      return i;
    }
  } //   namespace manipulation
} // namespace hpp
