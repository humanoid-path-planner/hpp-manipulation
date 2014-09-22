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

#include <hpp/core/straight-path.hh>

#include "hpp/manipulation/robot.hh"
#include "hpp/manipulation/graph/edge.hh"

namespace hpp {
  namespace manipulation {
    namespace graph {
      Edge::Edge () : pathConstraints_ (new Constraint_t()),
      configConstraints_ (new Constraint_t())
      {}

      Edge::~Edge ()
      {
        if (pathConstraints_  ) delete pathConstraints_;
        if (configConstraints_) delete configConstraints_;
      }

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

      ConstraintSetPtr_t Edge::configConstraint() const
      {
        if (!*configConstraints_) {
          configConstraints_->set (graph_.lock ()->configConstraint (wkPtr_.lock ()));
        }
        return configConstraints_->get ();
      }

      ConstraintSetPtr_t Edge::pathConstraint() const
      {
        if (!*pathConstraints_) {
          pathConstraints_->set (graph_.lock ()->pathConstraint (wkPtr_.lock ()));
        }
        return pathConstraints_->get ();
      }

      bool Edge::build (core::PathPtr_t& path, ConfigurationIn_t q1, ConfigurationIn_t q2, const core::WeighedDistance& d) const
      {
        ConstraintSetPtr_t constraints = pathConstraint ();
        constraints->offsetFromConfig(q1);
        if (!constraints->isSatisfied (q1) || !constraints->isSatisfied (q2)) {
          return false;
        }
        path = core::StraightPath::create (graph_.lock ()->robot (), q1, q2, d (q1, q2));
        path->constraints (constraints);
        return true;
      }
    } // namespace graph
  } // namespace manipulation
} // namespace hpp
