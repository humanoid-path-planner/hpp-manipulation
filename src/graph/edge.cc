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

#include "hpp/manipulation/graph/edge.hh"

namespace hpp {
  namespace manipulation {
    namespace graph {
      EdgePtr_t Edge::create (const NodeWkPtr_t& from, const NodeWkPtr_t& to)
      {
        Edge* ptr = new Edge;
        EdgePtr_t shPtr (ptr);
        ptr->init(shPtr, from, to);
        return shPtr;
      }

      void Edge::init (const EdgeWkPtr_t& weak, const NodeWkPtr_t& from,
          const NodeWkPtr_t& to)
      {
        GraphComponent::init (weak);
        wkPtr_ = weak;
        from_ = from;
        to_ = to;
        isInNodeFrom_ = false;
      }

      std::ostream& Edge::print (std::ostream& os) const
      {
        GraphComponent::print (os << "|   |   |-- ")
          << " --> " << to_.lock ()->name () << std::endl;
        return os;
      }

      ConstraintPtr_t Edge::configConstraint(ConfigurationIn_t config)
      {
        if (!configConstraints_) {
          configConstraints_ = graph_.lock ()->configConstraint (wkPtr_.lock ());
        }
        configConstraints_->offsetFromConfig (config);
        return configConstraints_;
      }

      ConstraintPtr_t Edge::pathConstraint(ConfigurationIn_t config)
      {
        if (!pathConstraints_) {
          pathConstraints_ = graph_.lock ()->pathConstraint (wkPtr_.lock ());
        }
        pathConstraints_->offsetFromConfig (config);
        return pathConstraints_;
      }
    } // namespace graph
  } // namespace manipulation
} // namespace hpp
