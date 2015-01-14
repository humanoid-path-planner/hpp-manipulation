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

#include <sstream>

#include <hpp/core/straight-path.hh>
#include <hpp/core/path-vector.hh>

#include <hpp/constraints/differentiable-function.hh>

#include <hpp/util/pointer.hh>

#include "hpp/manipulation/robot.hh"
#include "hpp/manipulation/graph/edge.hh"
#include "hpp/manipulation/graph/statistics.hh"

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

      NodePtr_t Edge::to () const
      {
        return to_.lock();
      }

      NodePtr_t Edge::from () const
      {
        return from_.lock();
      }

      NodePtr_t Edge::node () const
      {
        if (isInNodeFrom_) return from ();
        else return to ();
      }

      EdgePtr_t Edge::create (const GraphWkPtr_t& graph, const NodeWkPtr_t& from, const NodeWkPtr_t& to)
      {
        Edge* ptr = new Edge;
        EdgePtr_t shPtr (ptr);
        ptr->init(shPtr, graph, from, to);
        return shPtr;
      }

      void Edge::init (const EdgeWkPtr_t& weak, const GraphWkPtr_t& graph, const NodeWkPtr_t& from,
          const NodeWkPtr_t& to)
      {
        GraphComponent::init (weak);
        parentGraph (graph);
        wkPtr_ = weak;
        from_ = from;
        to_ = to;
        isInNodeFrom_ = false;
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
        da.addTooltipLine ("Edge constains:");
        populateTooltip (da);
        os << from()->id () << " -> " << to()->id () << " " << da << ";";
        return os;
      }

      ConstraintSetPtr_t Edge::configConstraint() const
      {
        if (!*configConstraints_) {
          configConstraints_->set (buildConfigConstraint ());
        }
        return configConstraints_->get ();
      }

      ConstraintSetPtr_t Edge::buildConfigConstraint() const
      {
        std::string n = "(" + name () + ")";
        GraphPtr_t g = graph_.lock ();

        ConstraintSetPtr_t constraint = ConstraintSet::create (g->robot (), "Set " + n);

        ConfigProjectorPtr_t proj = ConfigProjector::create(g->robot(), "proj_" + n, g->errorThreshold(), g->maxIterations());
        g->insertNumericalConstraints (proj);
        insertNumericalConstraints (proj);
        to ()->insertNumericalConstraints (proj);
        constraint->addConstraint (proj);

        g->insertLockedJoints (proj);
        insertLockedJoints (proj);
        to ()->insertLockedJoints (proj);
        return constraint;
      }

      ConstraintSetPtr_t Edge::pathConstraint() const
      {
        if (!*pathConstraints_) {
          pathConstraints_->set (buildPathConstraint ());
        }
        return pathConstraints_->get ();
      }

      ConstraintSetPtr_t Edge::buildPathConstraint() const
      {
        std::string n = "(" + name () + ")";
        GraphPtr_t g = graph_.lock ();

        ConstraintSetPtr_t constraint = ConstraintSet::create (g->robot (), "Set " + n);

        ConfigProjectorPtr_t proj = ConfigProjector::create(g->robot(), "proj_" + n, g->errorThreshold(), g->maxIterations());
        g->insertNumericalConstraints (proj);
        insertNumericalConstraints (proj);
        node ()->insertNumericalConstraintsForPath (proj);
        constraint->addConstraint (proj);

        g->insertLockedJoints (proj);
        insertLockedJoints (proj);
        node ()->insertLockedJoints (proj);
        return constraint;
      }

      bool Edge::build (core::PathPtr_t& path, ConfigurationIn_t q1, ConfigurationIn_t q2, const core::WeighedDistance& d) const
      {
        ConstraintSetPtr_t constraints = pathConstraint ();
        constraints->configProjector ()->rightHandSideFromConfig(q1);
        if (!constraints->isSatisfied (q1) || !constraints->isSatisfied (q2)) {
          return false;
        }
        path = core::StraightPath::create (graph_.lock ()->robot (), q1, q2, d (q1, q2));
        path->constraints (constraints);
        return true;
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
        if (c->apply (q)) {
          return true;
        }
	assert (proj);
	::hpp::statistics::SuccessStatistics& ss = proj->statistics ();
	if (ss.nbFailure () > ss.nbSuccess ()) {
	  hppDout (warning, c->name () << " fails often." << std::endl << ss);
	} else {
	  hppDout (warning, c->name () << " succeeds at rate "
		   << (double)(ss.nbSuccess ()) / ss.numberOfObservations ()
		   << ".");
	}
        return false;
      }

      WaypointEdgePtr_t WaypointEdge::create (const GraphWkPtr_t& graph, const NodeWkPtr_t& from, const NodeWkPtr_t& to)
      {
        WaypointEdge* ptr = new WaypointEdge;
        WaypointEdgePtr_t shPtr (ptr);
        ptr->init(shPtr, graph, from, to);
        return shPtr;
      }

      void WaypointEdge::init (const EdgeWkPtr_t& weak, const GraphWkPtr_t& graph, const NodeWkPtr_t& from,
          const NodeWkPtr_t& to)
      {
        Edge::init (weak, graph, from, to);
      }

      bool WaypointEdge::build (core::PathPtr_t& path, ConfigurationIn_t q1, ConfigurationIn_t q2, const core::WeighedDistance& d) const
      {
        assert (waypoint_.first);
        core::PathPtr_t pathToWaypoint;
        // Many times, this will be called rigth after WaypointEdge::applyConstraints so config_
        // already satisfies the constraints.
        if (!result_.isApprox (q2)) config_ = q2;
        if (!waypoint_.first->applyConstraints (q1, config_))
          return false;
        if (!waypoint_.first->build (pathToWaypoint, q1, config_, d))
          return false;
        core::PathVectorPtr_t pv = HPP_DYNAMIC_PTR_CAST (core::PathVector, pathToWaypoint);
        if (!pv) {
          pv = core::PathVector::create
	    (graph_.lock ()->robot ()->configSize (),
	     graph_.lock ()->robot ()->numberDof ());
          pv->appendPath (pathToWaypoint);
        }
        path = pv;

        core::PathPtr_t end;
        if (!Edge::build (end, config_, q2, d))
          return false;
        pv->appendPath (end);
        return true;
      }

      bool WaypointEdge::applyConstraints (ConfigurationIn_t qoffset, ConfigurationOut_t q) const
      {
        assert (waypoint_.first);
        config_ = q;
        if (!waypoint_.first->applyConstraints (qoffset, config_))
          return false;
        bool success = Edge::applyConstraints (config_, q);
        result_ = q;
        return success;
      }

      void WaypointEdge::createWaypoint (const unsigned d, const std::string& bname)
      {
        NodePtr_t node = Node::create ();
        node->parentGraph(graph_);
        std::ostringstream ss;
        ss << bname << "_n" << d;
        node->name (ss.str());
        EdgePtr_t edge;
        if (d == 0) {
          edge = Edge::create (graph_, from (), node);
          edge->isInNodeFrom (isInNodeFrom ());
        } else {
          WaypointEdgePtr_t we = WaypointEdge::create (graph_, from (), node);
          we->createWaypoint (d-1, bname);
          edge = we;
          edge->isInNodeFrom (isInNodeFrom ());
        }
        ss.str (std::string ()); ss.clear ();
        ss << bname << "_e" << d;
        edge->name (ss.str ());
        waypoint_ = Waypoint (edge, node);
        config_ = Configuration_t(graph_.lock ()->robot ()->configSize ());
        result_ = Configuration_t(graph_.lock ()->robot ()->configSize ());
      }

      NodePtr_t WaypointEdge::node () const
      {
        if (isInNodeFrom ()) return waypoint_.second;
        else return to ();
      }

      template <>
      EdgePtr_t WaypointEdge::waypoint <Edge> () const
      {
        return waypoint_.first;
      }

      template <>
      WaypointEdgePtr_t WaypointEdge::waypoint <WaypointEdge> () const
      {
        return HPP_DYNAMIC_PTR_CAST (WaypointEdge, waypoint_.first);
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
        waypoint_.second->dotPrint (os, da);
        da ["style"]="solid";
        waypoint_.first->dotPrint (os, da) << std::endl;
        da ["style"]="dotted";
        da ["dir"] = "both";
        da ["arrowtail"]="dot";
        da.insert ("shape", "onormal");
        da.insertWithQuote ("label", name());
        da.addTooltipLine ("Edge constains:");
        populateTooltip (da);
        os << waypoint_.second->id () << " -> " << to()->id () << " " << da << ";";
        return os;
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

      bool LevelSetEdge::applyConstraints (ConfigurationIn_t, ConfigurationOut_t) const
      {
        throw std::logic_error ("I need to know which connected component we wish to use.");
      }

      bool LevelSetEdge::applyConstraints (core::NodePtr_t n_offset, ConfigurationOut_t q) const
      {
        // First, get an offset from the histogram that is not in the same connected component.
        statistics::DiscreteDistribution < core::NodePtr_t > distrib = hist_->getDistribOutOfConnectedComponent (n_offset->connectedComponent ());
        const Configuration_t& levelsetTarget = *(distrib ()->configuration ()),
                               q_offset = *(n_offset->configuration ());
        // Then, set the offset.
        ConstraintSetPtr_t cs = extraConfigConstraint ();
        const ConfigProjectorPtr_t cp = cs->configProjector ();
        assert (cp);
	vector_t offset = cp->rightHandSideFromConfig (q_offset);
	size_t row = 0, nbRows = 0;
	for (NumericalConstraints_t::const_iterator it =
	       extraNumericalConstraints_.begin ();
	     it != extraNumericalConstraints_.end (); ++it) {
	  const core::DifferentiableFunction& f = (*it)->function ();
	  nbRows = f.outputSize ();
	  vector_t value = vector_t::Zero (nbRows);
	  // TODO: fix this function
	  if (!(*it)->comparisonType ()->constantRightHandSide ()) {
	    f (value, levelsetTarget);
            offset.segment (row, nbRows) = value;
            row += nbRows;
	  }
	}
	cp->rightHandSide (offset);
        for (LockedJoints_t::const_iterator it = extraLockedJoints_.begin ();
	     it != extraLockedJoints_.end (); ++it) {
          (*it)->rightHandSideFromConfig (levelsetTarget);
        }

        // Eventually, do the projection.
        if (cs->apply (q))
          return true;
	::hpp::statistics::SuccessStatistics& ss = cp->statistics ();
	if (ss.nbFailure () > ss.nbSuccess ()) {
	  hppDout (warning, cs->name () << " fails often." << std::endl << ss);
	} else {
	  hppDout (warning, cs->name () << " succeeds at rate "
		   << (double)(ss.nbSuccess ()) / ss.numberOfObservations ()
		   << ".");
	}
        return false;
      }

      void LevelSetEdge::init (const EdgeWkPtr_t& weak, const GraphWkPtr_t& graph, const NodeWkPtr_t& from,
          const NodeWkPtr_t& to)
      {
        Edge::init (weak, graph, from, to);
      }

      LevelSetEdgePtr_t LevelSetEdge::create (const GraphWkPtr_t& graph, const NodeWkPtr_t& from, const NodeWkPtr_t& to)
      {
        LevelSetEdge* ptr = new LevelSetEdge;
        LevelSetEdgePtr_t shPtr (ptr);
        ptr->init(shPtr, graph, from, to);
        return shPtr;
      }

      void LevelSetEdge::buildHistogram ()
      {
        std::string n = "(" + name () + ")";
        GraphPtr_t g = graph_.lock ();

        /// The order is important here for the offset.
        ConstraintSetPtr_t constraint = ConstraintSet::create (g->robot (), "Set " + n);

        ConfigProjectorPtr_t proj = ConfigProjector::create(g->robot(), "proj_" + n, g->errorThreshold(), g->maxIterations());
        for (NumericalConstraints_t::const_iterator it = extraNumericalConstraints_.begin ();
            it != extraNumericalConstraints_.end (); ++it) {
          proj->add (*it);
        }

        for (LockedJoints_t::const_iterator it = extraLockedJoints_.begin ();
            it != extraLockedJoints_.end (); ++it)
          proj->add (*it);

        constraint->addConstraint (proj);

        hist_ = graph::LeafHistogramPtr_t (new graph::LeafHistogram (constraint));
      }

      LeafHistogramPtr_t LevelSetEdge::histogram () const
      {
        return hist_;
      }

      ConstraintSetPtr_t LevelSetEdge::extraConfigConstraint () const
      {
        if (!*extraConstraints_) {
          std::string n = "(" + name () + "_extra)";
          GraphPtr_t g = graph_.lock ();

          /// The order is important here for the offset.
          ConstraintSetPtr_t constraint = ConstraintSet::create (g->robot (), "Set " + n);

          ConfigProjectorPtr_t proj = ConfigProjector::create(g->robot(), "proj_" + n, g->errorThreshold(), g->maxIterations());
          g->insertNumericalConstraints (proj);
          for (NumericalConstraints_t::const_iterator it = extraNumericalConstraints_.begin ();
              it != extraNumericalConstraints_.end (); ++it) {
            proj->add (*it);
          }
          !extraNumericalConstraints_.empty ();
          insertNumericalConstraints (proj);
          to ()->insertNumericalConstraints (proj);
          constraint->addConstraint (proj);

          g->insertLockedJoints (proj);
          for (LockedJoints_t::const_iterator it = extraLockedJoints_.begin ();
              it != extraLockedJoints_.end (); ++it) {
            proj->add (*it);
          }
          insertLockedJoints (proj);
          to ()->insertLockedJoints (proj);
          extraConstraints_->set (constraint);
        }
        return extraConstraints_->get ();
      }

      void LevelSetEdge::insertConfigConstraint (const NumericalConstraintPtr_t& nm)
      {
        extraNumericalConstraints_.push_back (nm);
      }

      void LevelSetEdge::insertConfigConstraint (const DifferentiableFunctionPtr_t function, const ComparisonTypePtr_t ineq)
      {
        insertConfigConstraint (NumericalConstraint::create (function, ineq));
      }

      void LevelSetEdge::insertConfigConstraint (const LockedJointPtr_t lockedJoint)
      {
        extraLockedJoints_.push_back (lockedJoint);
      }

      LevelSetEdge::LevelSetEdge (): extraConstraints_ (new Constraint_t()) {}

      LevelSetEdge::~LevelSetEdge ()
      {
        if (extraConstraints_  ) delete extraConstraints_;
      }
    } // namespace graph
  } // namespace manipulation
} // namespace hpp
