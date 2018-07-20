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

#include "hpp/manipulation/graph/graph-component.hh"

#include <hpp/core/config-projector.hh>
#include <hpp/core/constraint-set.hh>
#include <hpp/constraints/locked-joint.hh>
#include <hpp/constraints/implicit.hh>

#include <hpp/constraints/differentiable-function.hh>

#include "hpp/manipulation/graph/graph.hh"

namespace hpp {
  namespace manipulation {
    namespace graph {
      typedef constraints::Implicit Implicit;
      const std::string& GraphComponent::name() const
      {
        return name_;
      }

      std::ostream& GraphComponent::print (std::ostream& os) const
      {
        os << id () << " : " << name ();
        return os;
      }

      std::ostream& GraphComponent::dotPrint (std::ostream& os, dot::DrawingAttributes) const
      {
        os << id ();
        return os;
      }

      void GraphComponent::setDirty()
      {
        isInit_ = false;
      }

      void GraphComponent::addNumericalConstraint (const ImplicitPtr_t& nm,
          const segments_t& passiveDofs)
      {
        isInit_ = false;
        numericalConstraints_.push_back(nm);
        passiveDofs_.push_back (passiveDofs);
      }

      void GraphComponent::addNumericalConstraint (const DifferentiableFunctionPtr_t& function, const ComparisonTypes_t& ineq)
      {
        addNumericalConstraint (Implicit::create (function,ineq));
      }

      void GraphComponent::resetNumericalConstraints ()
      {
        isInit_ = false;
	numericalConstraints_.clear();
        passiveDofs_.clear();
      }

      void GraphComponent::addLockedJointConstraint
      (const LockedJointPtr_t& constraint)
      {
        isInit_ = false;
        lockedJoints_.push_back (constraint);
      }

      void GraphComponent::resetLockedJoints ()
      {
        isInit_ = false;
	lockedJoints_.clear();
      }

      bool GraphComponent::insertNumericalConstraints (ConfigProjectorPtr_t& proj) const
      {
        IntervalsContainer_t::const_iterator itpdof = passiveDofs_.begin ();
        for (NumericalConstraints_t::const_iterator it = numericalConstraints_.begin();
            it != numericalConstraints_.end(); ++it) {
          proj->add (*it, *itpdof);
          ++itpdof;
        }
        assert (itpdof == passiveDofs_.end ());
        return !numericalConstraints_.empty ();
      }

      bool GraphComponent::insertLockedJoints (ConfigProjectorPtr_t& cp) const
      {
        for (LockedJoints_t::const_iterator it = lockedJoints_.begin();
            it != lockedJoints_.end(); ++it)
          cp->add (*it);
        return !lockedJoints_.empty ();
      }

      const NumericalConstraints_t& GraphComponent::numericalConstraints() const
      {
        return numericalConstraints_;
      }

      const std::vector <segments_t>& GraphComponent::passiveDofs() const
      {
        return passiveDofs_;
      }

      const LockedJoints_t& GraphComponent::lockedJoints () const
      {
        return lockedJoints_;
      }

      GraphPtr_t GraphComponent::parentGraph() const
      {
        return graph_.lock ();
      }

      void GraphComponent::parentGraph(const GraphWkPtr_t& parent)
      {
        graph_ = parent;
        GraphPtr_t g = graph_.lock();
        assert(g);
        id_ = g->components().size();
        g->components().push_back (wkPtr_);
      }

      void GraphComponent::init (const GraphComponentWkPtr_t& weak)
      {
        wkPtr_ = weak;
      }

      void GraphComponent::throwIfNotInitialized () const
      {
        if (!isInit_ || (graph_.lock() && !graph_.lock()->isInit_)) throw std::logic_error ("The graph should have been initialized first.");
      }

      std::ostream& operator<< (std::ostream& os,
          const hpp::manipulation::graph::GraphComponent& graphComp)
      {
        return graphComp.print (os);
      }

      void GraphComponent::populateTooltip (dot::Tooltip& tp) const
      {
        for (NumericalConstraints_t::const_iterator it = numericalConstraints_.begin ();
            it != numericalConstraints_.end (); ++it) {
          tp.addLine ("- " + (*it)->function ().name ());
        }
        for (LockedJoints_t::const_iterator it = lockedJoints_.begin ();
            it != lockedJoints_.end (); ++it) {
          tp.addLine ("- " + (*it)->jointName ());
        }
      }
    } // namespace graph
  } // namespace manipulation
} // namespace hpp
