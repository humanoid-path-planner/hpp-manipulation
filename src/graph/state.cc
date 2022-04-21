// Copyright (c) 2014, LAAS-CNRS
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.

#include "hpp/manipulation/graph/state.hh"

#include <hpp/constraints/differentiable-function.hh>
#include <hpp/constraints/solver/by-substitution.hh>

#include "hpp/manipulation/device.hh"
#include "hpp/manipulation/graph/edge.hh"
#include "hpp/manipulation/graph/graph.hh"
#include "hpp/manipulation/constraint-set.hh"

namespace hpp {
  namespace manipulation {
    namespace graph {
      State::State (const std::string& name) :
	GraphComponent (name), configConstraints_ (),
        isWaypoint_ (false)
      {}

      State::~State ()
      {}

      StatePtr_t State::create (const std::string& name)
      {
        State* state = new State (name);
        StatePtr_t shPtr(state);
        shPtr->init(shPtr);
        return shPtr;
      }

      void State::init (const StateWkPtr_t& weak)
      {
        GraphComponent::init (weak);
        wkPtr_ = weak;
      }

      EdgePtr_t State::linkTo(const std::string& name, const StatePtr_t& to,
			     const size_type& w, EdgeFactory create)
      {
        EdgePtr_t newEdge = create(name, graph_, wkPtr_, to);
        if (w >= 0) neighbors_.insert (newEdge, (Weight_t)w);
        else hiddenNeighbors_.push_back (newEdge);
        return newEdge;
      }

      bool State::contains (ConfigurationIn_t config) const
      {
        return configConstraint()->isSatisfied (config);
      }

      std::ostream& State::dotPrint (std::ostream& os, dot::DrawingAttributes da) const
      {
        da.insertWithQuote ("label", name ());
        da.insert ("style","filled");
        dot::Tooltip tp; tp.addLine ("State contains:");
        populateTooltip (tp);
        da.insertWithQuote ("tooltip", tp.toStr());
        os << id () << " " << da << ";" << std::endl;

        dot::DrawingAttributes dac;
        std::vector <double> p = neighbors_.probabilities ();
        size_t i = 0;
        for (Neighbors_t::const_iterator it = neighbors_.begin();
            it != neighbors_.end(); ++it) {
          std::ostringstream oss; oss << (p[i] * 3 + 0.5);
          dac ["penwidth"] = oss.str ();
          i++;
          it->second->dotPrint (os, dac) << std::endl;
        }
        return os;
      }

      void State::populateTooltip (dot::Tooltip& tp) const
      {
        GraphComponent::populateTooltip (tp);
        tp.addLine ("");
        tp.addLine ("Numerical constraints for paths are:");
        for (NumericalConstraints_t::const_iterator it = numericalConstraintsForPath_.begin ();
            it != numericalConstraintsForPath_.end (); ++it) {
          tp.addLine ("- " + (*it)->function ().name ());
        }
      }

      std::ostream& State::print (std::ostream& os) const
      {
        os << "|   |-- ";
        GraphComponent::print (os) << std::endl;
        for (Neighbors_t::const_iterator it = neighbors_.begin();
            it != neighbors_.end(); ++it)
          os << *(it->second) << " - " << it->first << std::endl;
        return os;
      }

      void State::initialize()
      {
        isInit_ = true;

        std::string n = "(" + name () + ")";
        GraphPtr_t g = graph_.lock ();
        configConstraints_ = ConstraintSet::create (g->robot (), "Set " + n);

        ConfigProjectorPtr_t proj = ConfigProjector::create(g->robot(), "proj " + n, g->errorThreshold(), g->maxIterations());
        proj->solver().solveLevelByLevel(this->solveLevelByLevel());
        g->insertNumericalConstraints (proj);
        insertNumericalConstraints (proj);
        configConstraints_->addConstraint (proj);
      }

      void State::updateWeight (const EdgePtr_t& e, const Weight_t& w)
      {
        for (Neighbors_t::const_iterator it = neighbors_.begin();
            it != neighbors_.end(); ++it) {
          if (it->second == e) {
            /// Update the weights
            neighbors_.insert (e, w);
          }
        }
        hppDout (error, "Edge not found");
      }

      Weight_t State::getWeight (const EdgePtr_t& e)
      {
        for (Neighbors_t::const_iterator it = neighbors_.begin();
            it != neighbors_.end(); ++it)
          if (it->second == e) return it->first;
        for (std::vector<EdgePtr_t>::const_iterator it = hiddenNeighbors_.begin();
            it != hiddenNeighbors_.end(); ++it)
          if (*it == e) return -1;
        hppDout (error, "Edge not found");
        return 0;
      }

      void State::addNumericalConstraint(const ImplicitPtr_t& numConstraint)
      {
        ComparisonTypes_t comp(numConstraint->comparisonType());
        for(constraints::ComparisonType c : comp)
        {
          if (c == constraints::Equality)
          {
            throw std::logic_error("Failed to insert constraint \""
	     + numConstraint->function().name()
	     + "\" as a state constraint since it contains a comparison "
             + "of type Equality");
          }
        }
        GraphComponent::addNumericalConstraint(numConstraint);
      }

    } // namespace graph
  } // namespace manipulation
} // namespace hpp
