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

      void Graph::initialize ()
      {
        hists_.clear ();
        assert(components_.size() >= 1 && components_[0].lock() == wkPtr_.lock());
        for (std::size_t i = 1; i < components_.size(); ++i)
          components_[i].lock()->initialize();
        constraintsAndComplements_.clear ();
        isInit_ = true;
      }

      StateSelectorPtr_t Graph::createStateSelector (const std::string& name)
      {
        isInit_ = false;
        stateSelector_ = StateSelector::create (name);
        stateSelector_->parentGraph (wkPtr_);
        return stateSelector_;
      }

      void Graph::stateSelector (StateSelectorPtr_t ns)
      {
        isInit_ = false;
        stateSelector_ = ns;
        stateSelector_->parentGraph (wkPtr_);
      }

      void Graph::maxIterations (size_type iterations)
      {
        isInit_ = false;
        maxIterations_ = iterations;
      }

      size_type Graph::maxIterations () const
      {
        return maxIterations_;
      }

      void Graph::errorThreshold (const value_type& threshold)
      {
        isInit_ = false;
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
          setDirty();
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
          if (it->second->to () == to)
            edges.push_back (it->second);
        }
        return edges;
      }

      EdgePtr_t Graph::chooseEdge (RoadmapNodePtr_t from) const
      {
        return stateSelector_->chooseEdge (from);
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
	ConstraintSetPtr_t cs (configConstraint (edge));
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
        return edge->configConstraint ();
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
