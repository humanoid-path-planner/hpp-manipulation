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

#include <sstream>

#include <hpp/util/pointer.hh>
#include <hpp/util/exception-factory.hh>

#include <hpp/pinocchio/configuration.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/core/path-validation.hh>

#include <hpp/constraints/differentiable-function.hh>

#include "hpp/manipulation/device.hh"
#include "hpp/manipulation/problem.hh"
#include "hpp/manipulation/graph-steering-method.hh"
#include "hpp/manipulation/graph/statistics.hh"
#include "hpp/manipulation/constraint-set.hh"

namespace hpp {
  namespace manipulation {
    namespace graph {
      Edge::Edge (const std::string& name) :
	GraphComponent (name), isShort_ (false),
        pathConstraints_ (),
	configConstraints_ (),
        steeringMethod_ (),
        pathValidation_ ()
      {}

      Edge::~Edge ()
      {}

      StatePtr_t Edge::to () const
      {
        return to_.lock();
      }

      StatePtr_t Edge::from () const
      {
        return from_.lock();
      }

      bool Edge::direction (const core::PathPtr_t& path) const
      {
        Configuration_t q0 = path->initial (),
                        q1 = path->end ();
        const bool src_contains_q0 = from()->contains (q0);
        const bool dst_contains_q0 = to  ()->contains (q0);
        const bool src_contains_q1 = from()->contains (q1);
        const bool dst_contains_q1 = to  ()->contains (q1);
        // Karnaugh table:
        // 1 = forward, 0 = reverse, ? = I don't know, * = 0 or 1
        // s0s1 \ d0d1 | 00 | 01 | 11 | 10
        // 00          |  ? |  ? |  ? |  ?
        // 01          |  ? |  ? |  0 |  0
        // 11          |  ? |  1 |  * |  0
        // 10          |  ? |  1 |  1 |  1
        // 
        /// true if reverse
        if (   (!src_contains_q0 && !src_contains_q1)
            || (!dst_contains_q0 && !dst_contains_q1)
            || (!src_contains_q0 && !dst_contains_q0))
          HPP_THROW (std::runtime_error,
              "Edge " << name() << " does not seem to have generated path from"
              << pinocchio::displayConfig(q0) << " to "
              << pinocchio::displayConfig(q1)
              );
        return !(src_contains_q0 && (!src_contains_q1 || dst_contains_q1));
      }

      bool Edge::intersectionConstraint (const EdgePtr_t& other,
          ConfigProjectorPtr_t proj) const
      {
        GraphPtr_t g = graph_.lock ();
        
        g->insertNumericalConstraints (proj);
        insertNumericalConstraints (proj);
        state ()->insertNumericalConstraints (proj);

        g->insertLockedJoints (proj);
        insertLockedJoints (proj);
        state ()->insertLockedJoints (proj);

        if (wkPtr_.lock() == other) // No intersection to be computed.
          return false;

        bool stateB_Eq_stateA = (state() == other->state());

        other->insertNumericalConstraints (proj);
        if (!stateB_Eq_stateA) other->state()->insertNumericalConstraints (proj);
        other->insertLockedJoints (proj);
        if (!stateB_Eq_stateA) other->state()->insertLockedJoints (proj);

        return true;
      }

      EdgePtr_t Edge::create (const std::string& name,
			      const GraphWkPtr_t& graph,
			      const StateWkPtr_t& from, const StateWkPtr_t& to)
      {
        Edge* ptr = new Edge (name);
        EdgePtr_t shPtr (ptr);
        ptr->init(shPtr, graph, from, to);
        return shPtr;
      }

      void Edge::init (const EdgeWkPtr_t& weak, const GraphWkPtr_t& graph,
		       const StateWkPtr_t& from, const StateWkPtr_t& to)
      {
        GraphComponent::init (weak);
        parentGraph (graph);
        wkPtr_ = weak;
        from_ = from;
        to_ = to;
        state_ = to;
      }

      void Edge::initialize ()
      {
        configConstraints_ = buildConfigConstraint ();
        pathConstraints_ = buildPathConstraint ();
        isInit_ = true;
      }

      std::ostream& Edge::print (std::ostream& os) const
      {
        os << "|   |   |-- ";
        GraphComponent::print (os)
          << " --> " << to_.lock ()->name ();
        return os;
      }

      std::ostream& Edge::dotPrint (std::ostream& os, dot::DrawingAttributes da) const
      {
        da.insertWithQuote ("label", name ());
        da.insert ("shape", "onormal");
        dot::Tooltip tp; tp.addLine ("Edge constains:");
        populateTooltip (tp);
        da.insertWithQuote ("tooltip", tp.toStr());
        da.insertWithQuote ("labeltooltip", tp.toStr());
        os << from()->id () << " -> " << to()->id () << " " << da << ";";
        return os;
      }

      ConstraintSetPtr_t Edge::configConstraint() const
      {
        throwIfNotInitialized ();
        return configConstraints_;
      }

      ConstraintSetPtr_t Edge::buildConfigConstraint()
      {
        std::string n = "(" + name () + ")";
        GraphPtr_t g = graph_.lock ();

        ConstraintSetPtr_t constraint = ConstraintSet::create (g->robot (), "Set " + n);

        ConfigProjectorPtr_t proj = ConfigProjector::create(g->robot(), "proj_" + n, g->errorThreshold(), g->maxIterations());
        g->insertLockedJoints (proj);
        insertLockedJoints (proj);
        to ()->insertLockedJoints (proj);
	if (state () != to ()) {
	  state ()->insertLockedJoints (proj);
	}

        g->insertNumericalConstraints (proj);
        insertNumericalConstraints (proj);
        to ()->insertNumericalConstraints (proj);
	if (state () != to ()) {
	  state ()->insertNumericalConstraints (proj);
	}

        constraint->addConstraint (proj);
        constraint->edge (wkPtr_.lock ());
        return constraint;
      }

      ConstraintSetPtr_t Edge::pathConstraint() const
      {
        throwIfNotInitialized ();
        return pathConstraints_;
      }

      ConstraintSetPtr_t Edge::buildPathConstraint()
      {
        std::string n = "(" + name () + ")";
        GraphPtr_t g = graph_.lock ();

        ConstraintSetPtr_t constraint = ConstraintSet::create (g->robot (), "Set " + n);

        ConfigProjectorPtr_t proj = ConfigProjector::create(g->robot(), "proj_" + n, g->errorThreshold(), g->maxIterations());
        g->insertLockedJoints (proj);
        insertLockedJoints (proj);
        state ()->insertLockedJoints (proj);

        g->insertNumericalConstraints (proj);
        insertNumericalConstraints (proj);
        state ()->insertNumericalConstraintsForPath (proj);

        constraint->addConstraint (proj);
        constraint->edge (wkPtr_.lock ());

        // Build steering method
        const ProblemPtr_t& problem (g->problem());
        steeringMethod_ = problem->steeringMethod()->innerSteeringMethod()->copy();
        steeringMethod_->constraints (constraint);
        // Build path validation and relative motion matrix
        // TODO this path validation will not contain obstacles added after
        // its creation.
        pathValidation_ = problem->pathValidationFactory ();
        relMotion_ = RelativeMotion::matrix (g->robot());
        RelativeMotion::fromConstraint (relMotion_, g->robot(), constraint);
        pathValidation_->filterCollisionPairs (relMotion_);
        return constraint;
      }

      bool Edge::canConnect (ConfigurationIn_t q1, ConfigurationIn_t q2)
	const
      {
        ConstraintSetPtr_t constraints = pathConstraint ();
        constraints->configProjector ()->rightHandSideFromConfig(q1);
        if (!constraints->isSatisfied (q1) || !constraints->isSatisfied (q2)) {
          return false;
        }
        return true;
      }

      bool Edge::build (core::PathPtr_t& path, ConfigurationIn_t q1,
			ConfigurationIn_t q2)
	const
      {
        using pinocchio::displayConfig;
	if (!steeringMethod_) {
	  std::ostringstream oss;
	  oss << "No steering method set in edge " << name () << ".";
	  throw std::runtime_error (oss.str ().c_str ());
	}
        ConstraintSetPtr_t constraints = pathConstraint ();
        constraints->configProjector ()->rightHandSideFromConfig(q1);
        if (constraints->isSatisfied (q1)) {
          if (constraints->isSatisfied (q2)) {
            path = (*steeringMethod_) (q1, q2);
            return (bool)path;
          } else {
	    hppDout(info, "q2 = " << displayConfig (q2)
                    << " does not satisfy the constraints of edge "
                    << name ());
	    return false;
	  }
        } else {
	  std::ostringstream oss;
	  oss << "The initial configuration " << displayConfig (q1)
              << " does not satisfy the constraints of"
	    " edge " << name () << "." << std::endl;
	  oss << "The graph is probably malformed";
	  throw std::runtime_error (oss.str ().c_str ());
	}
      }

      bool Edge::applyConstraints (core::NodePtr_t nnear, ConfigurationOut_t q) const
      {
        return applyConstraints (*(nnear->configuration ()), q);
      }

      bool Edge::applyConstraints (ConfigurationIn_t qoffset,
				   ConfigurationOut_t q) const
      {
        ConstraintSetPtr_t c = configConstraint ();
        ConfigProjectorPtr_t proj = c->configProjector ();
        proj->rightHandSideFromConfig (qoffset);
        if (isShort_) q = qoffset;
        if (c->apply (q)) return true;
	const ::hpp::statistics::SuccessStatistics& ss = proj->statistics ();
	if (ss.nbFailure () > ss.nbSuccess ()) {
	  hppDout (warning, c->name () << " fails often.\n" << ss);
	} else {
	  hppDout (warning, c->name () << " succeeds at rate "
		   << (value_type)(ss.nbSuccess ()) /
		   (value_type) ss.numberOfObservations ()
		   << ".");
	}
        return false;
      }

      WaypointEdgePtr_t WaypointEdge::create (const std::string& name,
       const GraphWkPtr_t& graph, const StateWkPtr_t& from,
       const StateWkPtr_t& to)
      {
        WaypointEdge* ptr = new WaypointEdge (name);
        WaypointEdgePtr_t shPtr (ptr);
        ptr->init(shPtr, graph, from, to);
        return shPtr;
      }

      void WaypointEdge::init (const WaypointEdgeWkPtr_t& weak,
			       const GraphWkPtr_t& graph,
			       const StateWkPtr_t& from,
			       const StateWkPtr_t& to)
      {
        Edge::init (weak, graph, from, to);
        nbWaypoints(0);
        wkPtr_ = weak;
      }

      bool WaypointEdge::canConnect (ConfigurationIn_t q1, ConfigurationIn_t q2) const
      {
        /// TODO: This is not correct
        for (std::size_t i = 0; i < edges_.size (); ++i)
          if (!edges_[i]->canConnect(q1, q2)) return false;
        return true;
      }

      bool WaypointEdge::build (core::PathPtr_t& path, ConfigurationIn_t q1,
          ConfigurationIn_t q2) const
      {
        core::PathPtr_t p;
        core::PathVectorPtr_t pv = core::PathVector::create
          (graph_.lock ()->robot ()->configSize (),
           graph_.lock ()->robot ()->numberDof ());
        // Many times, this will be called rigth after WaypointEdge::applyConstraints so config_
        // already satisfies the constraints.
        size_type n = edges_.size() - 1;
        bool useCache = configs_.col(0).isApprox (q1)
          &&            configs_.col(n).isApprox (q2);
        if (!useCache) {
          configs_.col(0) = q1;
          configs_.col(n) = q2;
        }

        for (std::size_t i = 0; i < edges_.size (); ++i) {
          if (!useCache) configs_.col (i+1) = q2;
          if (!edges_[i]->applyConstraints (configs_.col(i), configs_.col (i+1))) {
            hppDout (info, "Waypoint edge " << name() << ": applyConstraints failed at waypoint " << i << "."
                << "\nUse cache: " << useCache
                );
            return false;
          }
          if (!edges_[i]->build (p, configs_.col(i), configs_.col (i+1))) {
            hppDout (info, "Waypoint edge " << name() << ": build failed at waypoint " << i << "."
                << "\nUse cache: " << useCache
                );
            return false;
          }
          pv->appendPath (p);
        }

        path = pv;
        return true;
      }

      bool WaypointEdge::applyConstraints (ConfigurationIn_t qoffset, ConfigurationOut_t q) const
      {
        assert (configs_.cols() == size_type(edges_.size() + 1));
        configs_.col(0) = qoffset;
        for (std::size_t i = 0; i < edges_.size (); ++i) {
          configs_.col (i+1) = q;
          if (!edges_[i]->applyConstraints (configs_.col(i), configs_.col (i+1))) {
            q = configs_.col(i+1);
            return false;
          }
        }
        q = configs_.col(edges_.size());
        return true;
      }

      void WaypointEdge::nbWaypoints (const size_type number)
      {
        edges_.resize (number + 1);
        states_.resize (number + 1);
        states_.back() = to();
        const size_type nbDof = graph_.lock ()->robot ()->configSize ();
        configs_ = matrix_t (nbDof, number + 2);
      }

      void WaypointEdge::setWaypoint (const std::size_t index,
				      const EdgePtr_t wEdge,
				      const StatePtr_t wTo)
      {
        assert (edges_.size() == states_.size());
        assert (index < edges_.size());
        if (index == states_.size() - 1) {
          assert (!wTo || wTo == to());
        } else {
          states_[index] = wTo;
        }
        edges_[index] = wEdge;
      }

      template <>
      EdgePtr_t WaypointEdge::waypoint <Edge> (const std::size_t index) const
      {
        assert (index < edges_.size()); 
        return edges_[index];
      }

      template <>
      WaypointEdgePtr_t WaypointEdge::waypoint <WaypointEdge> (const std::size_t index) const
      {
        assert (index < edges_.size()); 
        return HPP_DYNAMIC_PTR_CAST (WaypointEdge, edges_[index]);
      }

      std::ostream& WaypointEdge::print (std::ostream& os) const
      {
        os << "|   |   |-- ";
        GraphComponent::print (os)
          << " (waypoint) --> " << to ()->name ();
        return os;
      }

      std::ostream& WaypointEdge::dotPrint (std::ostream& os, dot::DrawingAttributes da) const
      {
        // First print the waypoint node, then the first edge.
        da ["style"]="dashed";
        for (std::size_t i = 0; i < states_.size () - 1; ++i)
          states_[i]->dotPrint (os, da);

        da ["style"]="solid";
        for (std::size_t i = 0; i < edges_.size (); ++i)
          edges_[i]->dotPrint (os, da) << std::endl;

        da ["style"]="dotted";
        da ["dir"] = "both";
        da ["arrowtail"]="dot";

        return Edge::dotPrint (os, da);
      }

      std::ostream& LevelSetEdge::print (std::ostream& os) const
      {
        os << "|   |   |-- ";
        GraphComponent::print (os)
          << " (level set) --> " << to ()->name ();
        return os;
      }

      std::ostream& LevelSetEdge::dotPrint (std::ostream& os, dot::DrawingAttributes da) const
      {
        da.insert ("shape", "onormal");
        da.insert ("style", "dashed");
        return Edge::dotPrint (os, da);
      }

      void LevelSetEdge::populateTooltip (dot::Tooltip& tp) const
      {
        GraphComponent::populateTooltip (tp);
        tp.addLine ("");
        tp.addLine ("Foliation condition constraints:");
        for (NumericalConstraints_t::const_iterator it = condNumericalConstraints_.begin ();
            it != condNumericalConstraints_.end (); ++it) {
          tp.addLine ("- " + (*it)->function ().name ());
        }
        for (LockedJoints_t::const_iterator it = condLockedJoints_.begin ();
            it != condLockedJoints_.end (); ++it) {
          tp.addLine ("- " + (*it)->jointName ());
        }
        tp.addLine ("Foliation parametrization constraints:");
        for (NumericalConstraints_t::const_iterator it = paramNumericalConstraints_.begin ();
            it != paramNumericalConstraints_.end (); ++it) {
          tp.addLine ("- " + (*it)->function ().name ());
        }
        for (LockedJoints_t::const_iterator it = paramLockedJoints_.begin ();
            it != paramLockedJoints_.end (); ++it) {
          tp.addLine ("- " + (*it)->jointName ());
        }
      }

      bool LevelSetEdge::applyConstraints (ConfigurationIn_t qoffset, ConfigurationOut_t q) const
      {
        // First, get an offset from the histogram
        statistics::DiscreteDistribution < RoadmapNodePtr_t > distrib = hist_->getDistrib ();
        if (distrib.size () == 0) {
          hppDout (warning, "Edge " << name() << ": Distrib is empty");
          return false;
        }
        const Configuration_t& qlevelset = *(distrib ()->configuration ());

        return applyConstraintsWithOffset (qoffset, qlevelset, q);
      }

      bool LevelSetEdge::applyConstraints (core::NodePtr_t n_offset, ConfigurationOut_t q) const
      {
        // First, get an offset from the histogram that is not in the same connected component.
        statistics::DiscreteDistribution < RoadmapNodePtr_t > distrib = hist_->getDistribOutOfConnectedComponent (n_offset->connectedComponent ());
        if (distrib.size () == 0) {
          hppDout (warning, "Edge " << name() << ": Distrib is empty");
          return false;
        }
        const Configuration_t& qlevelset = *(distrib ()->configuration ()),
                               qoffset = *(n_offset->configuration ());

        return applyConstraintsWithOffset (qoffset, qlevelset, q);
      }

      bool LevelSetEdge::applyConstraintsWithOffset (ConfigurationIn_t qoffset,
          ConfigurationIn_t qlevelset, ConfigurationOut_t q) const
      {
        // First, set the offset.
        ConstraintSetPtr_t cs = configConstraint ();
        const ConfigProjectorPtr_t cp = cs->configProjector ();
        assert (cp);

	cp->rightHandSideFromConfig (qoffset);
	for (NumericalConstraints_t::const_iterator it =
	       paramNumericalConstraints_.begin ();
	     it != paramNumericalConstraints_.end (); ++it) {
          cp->rightHandSideFromConfig (*it, qlevelset);
        }
        for (LockedJoints_t::const_iterator it = paramLockedJoints_.begin ();
	     it != paramLockedJoints_.end (); ++it) {
          cp->rightHandSideFromConfig (*it, qlevelset);
        }

        // Eventually, do the projection.
        if (isShort_) q = qoffset;
        if (cs->apply (q)) return true;
	::hpp::statistics::SuccessStatistics& ss = cp->statistics ();
	if (ss.nbFailure () > ss.nbSuccess ()) {
	  hppDout (warning, cs->name () << " fails often." << std::endl << ss);
	} else {
	  hppDout (warning, cs->name () << " succeeds at rate "
		   << (value_type)(ss.nbSuccess ()) /
		   (value_type) ss.numberOfObservations ()
		   << ".");
	}
        return false;
      }

      void LevelSetEdge::init (const LevelSetEdgeWkPtr_t& weak,
			       const GraphWkPtr_t& graph,
			       const StateWkPtr_t& from,
			       const StateWkPtr_t& to)
      {
        Edge::init (weak, graph, from, to);
        wkPtr_ = weak;
      }

      LevelSetEdgePtr_t LevelSetEdge::create
      (const std::string& name, const GraphWkPtr_t& graph,
       const StateWkPtr_t& from, const StateWkPtr_t& to)
      {
        LevelSetEdge* ptr = new LevelSetEdge (name);
        LevelSetEdgePtr_t shPtr (ptr);
        ptr->init(shPtr, graph, from, to);
        return shPtr;
      }

      LeafHistogramPtr_t LevelSetEdge::histogram () const
      {
        return hist_;
      }

      void LevelSetEdge::buildHistogram ()
      {
        Foliation f;

        /// Build the constraint set.
        std::string n = "(" + name () + ")";
        GraphPtr_t g = graph_.lock ();

        // The parametrizer
        ConstraintSetPtr_t param = ConstraintSet::create (g->robot (), "Set " + n);

        ConfigProjectorPtr_t proj = ConfigProjector::create(g->robot(), "projParam_" + n, g->errorThreshold(), g->maxIterations());
        IntervalsContainer_t::const_iterator itpdof = paramPassiveDofs_.begin ();
        for (NumericalConstraints_t::const_iterator it = paramNumericalConstraints_.begin ();
            it != paramNumericalConstraints_.end (); ++it) {
          proj->add (*it, *itpdof);
          ++itpdof;
        }
        assert (itpdof == paramPassiveDofs_.end ());

        param->addConstraint (proj);
        param->edge (wkPtr_.lock ());

        for (LockedJoints_t::const_iterator it = paramLockedJoints_.begin ();
            it != paramLockedJoints_.end (); ++it) {
          proj->add (*it);
        }

        f.parametrizer (param);

        // The codition
        // TODO: We assumed that this part of the code can only be reached by
        // configurations that are valid.
        // It would be wiser to make sure configurations are valid, for instance
        // by considering only configurations in the destination state of this
        // edge.
        ConstraintSetPtr_t cond = ConstraintSet::create (g->robot (), "Set " + n);
        proj = ConfigProjector::create(g->robot(), "projCond_" + n, g->errorThreshold(), g->maxIterations());
        itpdof = condPassiveDofs_.begin ();
        for (NumericalConstraints_t::const_iterator it = condNumericalConstraints_.begin ();
            it != condNumericalConstraints_.end (); ++it) {
          proj->add (*it, *itpdof);
          ++itpdof;
        }
        assert (itpdof == condPassiveDofs_.end ());
        for (LockedJoints_t::const_iterator it = condLockedJoints_.begin ();
            it != condLockedJoints_.end (); ++it) {
          proj->add (*it);
        }
        f.condition (cond);
        cond->addConstraint (proj);

        hppDout(info, "Build histogram of LevelSetEdge " << name()
            << "\nParametrizer:\n" << *param
            << "\nCondition:\n" << *cond
            );

        // TODO: If hist_ is not NULL, remove the previous Histogram.
        // It should not be of any use and it slows down node insertion in the
        // roadmap.
        hist_ = LeafHistogram::create (f);
        g->insertHistogram (hist_);
      }

      void LevelSetEdge::initialize ()
      {
        Edge::initialize();
        buildHistogram ();
      }

      ConstraintSetPtr_t LevelSetEdge::buildConfigConstraint()
      {
        std::string n = "(" + name () + ")";
        GraphPtr_t g = graph_.lock ();

        ConstraintSetPtr_t constraint = ConstraintSet::create (g->robot (), "Set " + n);

        ConfigProjectorPtr_t proj = ConfigProjector::create(g->robot(), "proj_" + n, g->errorThreshold(), g->maxIterations());
        g->insertNumericalConstraints (proj);
        IntervalsContainer_t::const_iterator itpdof = paramPassiveDofs_.begin ();
        for (NumericalConstraints_t::const_iterator it = paramNumericalConstraints_.begin ();
            it != paramNumericalConstraints_.end (); ++it) {
          proj->add (*it, *itpdof);
          ++itpdof;
        }
        assert (itpdof == paramPassiveDofs_.end ());

        insertNumericalConstraints (proj);
        to ()->insertNumericalConstraints (proj);
        if (state () != to ()) {
	  state ()->insertNumericalConstraints (proj);
	}
        constraint->addConstraint (proj);

        g->insertLockedJoints (proj);
        for (LockedJoints_t::const_iterator it = paramLockedJoints_.begin ();
            it != paramLockedJoints_.end (); ++it) {
          proj->add (*it);
        }
        insertLockedJoints (proj);
        to ()->insertLockedJoints (proj);
        if (state () != to ()) {
	  state ()->insertLockedJoints (proj);
	}

        constraint->edge (wkPtr_.lock ());
        return constraint;
      }

      void LevelSetEdge::insertParamConstraint (const NumericalConstraintPtr_t& nm,
              const SizeIntervals_t& passiveDofs)
      {
        isInit_ = false;
        paramNumericalConstraints_.push_back (nm);
        paramPassiveDofs_.push_back (passiveDofs);
      }

      void LevelSetEdge::insertParamConstraint (const DifferentiableFunctionPtr_t function, const ComparisonTypePtr_t ineq)
      {
        isInit_ = false;
        insertParamConstraint (NumericalConstraint::create (function, ineq));
      }

      void LevelSetEdge::insertParamConstraint (const LockedJointPtr_t lockedJoint)
      {
        isInit_ = false;
        paramLockedJoints_.push_back (lockedJoint);
      }

      void LevelSetEdge::insertConditionConstraint (const NumericalConstraintPtr_t& nm,
              const SizeIntervals_t& passiveDofs)
      {
        isInit_ = false;
        condNumericalConstraints_.push_back (nm);
        condPassiveDofs_.push_back (passiveDofs);
      }

      void LevelSetEdge::insertConditionConstraint (const LockedJointPtr_t lockedJoint)
      {
        isInit_ = false;
        condLockedJoints_.push_back (lockedJoint);
      }

      LevelSetEdge::LevelSetEdge
      (const std::string& name) :
	Edge (name)
      {
      }

      LevelSetEdge::~LevelSetEdge ()
      {}
    } // namespace graph
  } // namespace manipulation
} // namespace hpp
