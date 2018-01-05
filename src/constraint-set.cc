// Copyright (c) 2015, Joseph Mirabel
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

#include "hpp/manipulation/constraint-set.hh"

#include "hpp/manipulation/device.hh"
#include "hpp/manipulation/graph/edge.hh"

namespace hpp {
  namespace manipulation {
    ConstraintSetPtr_t ConstraintSet::create
      (const DevicePtr_t& robot, const std::string& name)
      {
        ConstraintSet* ptr = new ConstraintSet (robot, name);
        ConstraintSetPtr_t shPtr (ptr);
        ptr->init (shPtr);
        return shPtr;
      }

    ConstraintSetPtr_t ConstraintSet::createCopy (const ConstraintSetPtr_t& cs)
    {
      ConstraintSet* ptr = new ConstraintSet (*cs);
      ConstraintSetPtr_t shPtr (ptr);
      ptr->init (shPtr);
      return shPtr;
    }

    ConstraintPtr_t ConstraintSet::copy () const
    {
      return createCopy (weak_.lock ());
    }

    void ConstraintSet::edge (graph::EdgePtr_t edge)
    {
      edge_ = edge;
    }

    graph::EdgePtr_t ConstraintSet::edge () const
    {
      return edge_;
    }

    ConstraintSet::ConstraintSet (const DevicePtr_t& robot, const std::string& name) :
      Parent_t (robot, name),
      edge_ ()
    {}

    ConstraintSet::ConstraintSet (const ConstraintSet& other) :
      Parent_t (other),
      edge_ (other.edge ())
    {}

    void ConstraintSet::init (const ConstraintSetPtr_t& self)
    {
      Parent_t::init (self);
      weak_ = self;
    }

    std::ostream& ConstraintSet::print (std::ostream& os) const
    {
      Parent_t::print (os);
      if (edge_) os << iendl << "Built by edge " << edge_->name();
      return os;
    }
  } // namespace manipulation
} // namespace hpp
