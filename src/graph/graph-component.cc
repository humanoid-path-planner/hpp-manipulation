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
#include <hpp/core/locked-joint.hh>

#include <hpp/constraints/differentiable-function.hh>

namespace hpp {
  namespace manipulation {
    namespace graph {
      std::vector < GraphComponentWkPtr_t > GraphComponent::components_ = std::vector < GraphComponentWkPtr_t >();

      const std::string& GraphComponent::name() const
      {
        return name_;
      }

      void GraphComponent::name(const std::string& name)
      {
        name_ = name;
      }

      GraphComponentWkPtr_t GraphComponent::get(std::size_t id)
      {
# ifdef HPP_DEBUG
        if (id >= components_.size())
          throw std::out_of_range ("ID out of range.");
# endif // HPP_DEBUG
        return components_[id];
      }

      const std::vector <GraphComponentWkPtr_t>& GraphComponent::components ()
      {
        return components_;
      }


      int GraphComponent::id () const
      {
        return id_;
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

      void GraphComponent::addNumericalConstraint (const NumericalConstraintPtr_t& nm,
          const SizeIntervals_t& passiveDofs)
      {
        numericalConstraints_.push_back(nm);
        passiveDofs_.push_back (passiveDofs);
      }

      void GraphComponent::addNumericalConstraint (const DifferentiableFunctionPtr_t& function, const ComparisonTypePtr_t& ineq)
      {
        addNumericalConstraint (NumericalConstraint::create (function,ineq));
      }

      void GraphComponent::addLockedJointConstraint
      (const LockedJointPtr_t& constraint)
      {
        lockedJoints_.push_back (constraint);
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

      const LockedJoints_t& GraphComponent::lockedJoints () const
      {
        return lockedJoints_;
      }

      void GraphComponent::parentGraph(const GraphWkPtr_t& parent)
      {
        graph_ = parent;
      }

      void GraphComponent::init (const GraphComponentWkPtr_t& weak)
      {
        wkPtr_ = weak;
        id_ = components_.size();
        components_.push_back (wkPtr_);
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
