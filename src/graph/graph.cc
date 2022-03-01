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

#include "hpp/manipulation/graph/graph.hh"

#include <hpp/util/assertion.hh>

#include <hpp/manipulation/constraint-set.hh>
#include "hpp/manipulation/graph/state-selector.hh"
#include "hpp/manipulation/graph/state.hh"
#include "hpp/manipulation/graph/edge.hh"
#include "hpp/manipulation/graph/statistics.hh"

namespace hpp {
  namespace manipulation {
    namespace graph {
      typedef constraints::Implicit Implicit;
      typedef constraints::ImplicitPtr_t ImplicitPtr_t;
      GraphPtr_t Graph::create(const std::string& name, DevicePtr_t robot,
			       const ProblemPtr_t& problem)
      {
        Graph* ptr = new Graph (name, problem);
        GraphPtr_t shPtr (ptr);
        ptr->init (shPtr, robot);
        shPtr->createStateSelector (name);
        return shPtr;
      }

      void Graph::init (const GraphWkPtr_t& weak, DevicePtr_t robot)
      {
        GraphComponent::init (weak);
        robot_ = robot;
        wkPtr_ = weak;
        parentGraph(wkPtr_);
        insertHistogram(graph::HistogramPtr_t (
              new graph::StateHistogram (wkPtr_.lock()))
            );
      }

      void Graph::initialize()
      {
        hists_.clear ();
        assert(components_.size() >= 1 && components_[0].lock() == wkPtr_.lock());
        for (std::size_t i = 1; i < components_.size(); ++i)
          components_[i].lock()->initialize();
        isInit_ = true;
      }

      void Graph::invalidate ()
      {
        for (std::size_t i = 1; i < components_.size(); ++i)
        {
          assert(components_[i].lock());
          components_[i].lock()->invalidate();
        }
        isInit_ = false;
      }

      StateSelectorPtr_t Graph::createStateSelector (const std::string& name)
      {
        invalidate ();
        stateSelector_ = StateSelector::create (name);
        stateSelector_->parentGraph (wkPtr_);
        return stateSelector_;
      }

      void Graph::stateSelector (StateSelectorPtr_t ns)
      {
        invalidate ();
        stateSelector_ = ns;
        stateSelector_->parentGraph (wkPtr_);
      }

      void Graph::maxIterations (size_type iterations)
      {
        invalidate ();
        maxIterations_ = iterations;
      }

      size_type Graph::maxIterations () const
      {
        return maxIterations_;
      }

      void Graph::errorThreshold (const value_type& threshold)
      {
        invalidate ();
        errorThreshold_ = threshold;
      }

      value_type Graph::errorThreshold () const
      {
        return errorThreshold_;
      }

      const DevicePtr_t& Graph::robot () const
      {
        return robot_;
      }

      void Graph::problem (const ProblemPtr_t& problem)
      {
        if (problem_ != problem) {
          problem_ = problem;
          invalidate();
        }
      }

      const ProblemPtr_t& Graph::problem () const
      {
	return problem_;
      }

      StatePtr_t Graph::getState (ConfigurationIn_t config) const
      {
        if (!stateSelector_) throw std::runtime_error ("No StateSelector in Constraint Graph.");
        return stateSelector_->getState (config);
      }

      StatePtr_t Graph::getState(RoadmapNodePtr_t coreNode) const
      {
        return stateSelector_->getState (coreNode);
      }

      Edges_t Graph::getEdges (const StatePtr_t& from, const StatePtr_t& to)
	const
      {
        Edges_t edges;
        for (Neighbors_t::const_iterator it = from->neighbors ().begin ();
            it != from->neighbors ().end (); ++it) {
          if (it->second->stateTo () == to)
            edges.push_back (it->second);
        }
        return edges;
      }

      EdgePtr_t Graph::chooseEdge (RoadmapNodePtr_t from) const
      {
        return stateSelector_->chooseEdge (from);
      }

      void Graph::clearConstraintsAndComplement()
      {
        constraintsAndComplements_.clear();
      }

      void Graph::registerConstraints
      (const ImplicitPtr_t& constraint,
       const ImplicitPtr_t& complement,
       const ImplicitPtr_t& both)
      {
        for (ConstraintsAndComplements_t::const_iterator it
               (constraintsAndComplements_.begin ());
             it != constraintsAndComplements_.end (); ++it) {
          assert (it->constraint != constraint);
        }
        constraintsAndComplements_.push_back (ConstraintAndComplement_t
                                              (constraint, complement, both));
      }

      bool Graph::isComplement (const ImplicitPtr_t& constraint,
                                const ImplicitPtr_t& complement,
                                ImplicitPtr_t& combinationOfBoth)
        const
      {
        for (ConstraintsAndComplements_t::const_iterator it =
               constraintsAndComplements_.begin ();
             it != constraintsAndComplements_.end (); ++it) {
          if ((it->constraint->functionPtr () == constraint->functionPtr ()) &&
              (it->complement->functionPtr () == complement->functionPtr ())) {
            combinationOfBoth = it->both;
            return true;
          }
        }
        return false;
      }

      const ConstraintsAndComplements_t& Graph::constraintsAndComplements ()
        const
      {
        return constraintsAndComplements_;
      }

      ConstraintSetPtr_t Graph::configConstraint (const StatePtr_t& state) const
      {
        return state->configConstraint ();
      }

      bool Graph::getConfigErrorForState (ConfigurationIn_t config,
					  const StatePtr_t& state,
					  vector_t& error) const
      {
	return configConstraint (state)->isSatisfied (config, error);
      }

      bool Graph::getConfigErrorForEdge (ConfigurationIn_t config,
					 const EdgePtr_t& edge, vector_t& error) const
      {
	ConstraintSetPtr_t cs (pathConstraint (edge));
	ConfigProjectorPtr_t cp (cs->configProjector ());
	if (cp) cp->rightHandSideFromConfig (config);
	return cs->isSatisfied (config, error);
      }

      bool Graph::getConfigErrorForEdgeTarget
      (ConfigurationIn_t leafConfig, ConfigurationIn_t config,
       const EdgePtr_t& edge, vector_t& error) const
      {
	ConstraintSetPtr_t cs (targetConstraint (edge));
	ConfigProjectorPtr_t cp (cs->configProjector ());
	if (cp) cp->rightHandSideFromConfig (leafConfig);
	return cs->isSatisfied (config, error);
      }

      bool Graph::getConfigErrorForEdgeLeaf
      (ConfigurationIn_t leafConfig, ConfigurationIn_t config,
       const EdgePtr_t& edge, vector_t& error) const
      {
	ConstraintSetPtr_t cs (pathConstraint (edge));
	ConfigProjectorPtr_t cp (cs->configProjector ());
	if (cp) cp->rightHandSideFromConfig (leafConfig);
	return cs->isSatisfied (config, error);
      }

      ConstraintSetPtr_t Graph::configConstraint (const EdgePtr_t& edge) const
      {
        return edge->targetConstraint ();
      }

      ConstraintSetPtr_t Graph::targetConstraint (const EdgePtr_t& edge) const
      {
        return edge->targetConstraint ();
      }

      ConstraintSetPtr_t Graph::pathConstraint (const EdgePtr_t& edge) const
      {
        return edge->pathConstraint ();
      }

      GraphComponentWkPtr_t Graph::get(std::size_t id) const
      {
        if (id >= components_.size())
          throw std::out_of_range ("ID out of range.");
        return components_[id];
      }

      GraphComponents_t& Graph::components ()
      {
        return components_;
      }

      Graph::Graph (const std::string& name, const ProblemPtr_t& problem) :
        GraphComponent (name), problem_ (problem)
      {
      }

      std::ostream& Graph::dotPrint (std::ostream& os, dot::DrawingAttributes da) const
      {
        da.separator = "; ";
        da.openSection = "\n";
        da.closeSection = ";\n";
        dot::Tooltip tp; tp.addLine ("Graph constains:");
        populateTooltip (tp);
        da.insertWithQuote ("tooltip", tp.toStr());
        os << "digraph " << id() << " {" << da;
        stateSelector_->dotPrint (os);
        os << "}" << std::endl;
        return os;
      }

      std::ostream& Graph::print (std::ostream& os) const
      {
        return GraphComponent::print (os) << std::endl << *stateSelector_;
      }
    } // namespace graph
  } // namespace manipulation

} // namespace hpp
