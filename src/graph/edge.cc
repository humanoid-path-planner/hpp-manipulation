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
#include <hpp/core/differentiable-function.hh>

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

      std::ostream& Edge::dotPrint (std::ostream& os) const
      {
        os << from()->id () << " -> " << to()->id () << " [shape=onormal,label=\"" << name () << "\"];";
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
        // If at least one DifferentiableFunctionPtr_t was inserted, the add the projector.
        if (g->insertNumericalConstraints (proj)
            | insertNumericalConstraints (proj)
            | to ()->insertNumericalConstraints (proj))
          constraint->addConstraint (proj);

        g->insertLockedDofs (constraint);
        insertLockedDofs (constraint);
        to ()->insertLockedDofs (constraint);
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
        // If at least one DifferentiableFunctionPtr_t was inserted, the add the projector.
        if (g->insertNumericalConstraints (proj)
            | insertNumericalConstraints (proj)
            | node ()->insertNumericalConstraintsForPath (proj))
          constraint->addConstraint (proj);

        g->insertLockedDofs (constraint);
        insertLockedDofs (constraint);
        node ()->insertLockedDofs (constraint);
        return constraint;
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

      bool Edge::applyConstraints (core::NodePtr_t nnear, ConfigurationOut_t q) const
      {
        return applyConstraints (*(nnear->configuration ()), q);
      }

      bool Edge::applyConstraints (ConfigurationIn_t qoffset, ConfigurationOut_t q) const
      {
        ConstraintSetPtr_t c = configConstraint ();
        c->offsetFromConfig (qoffset);
        ConfigProjectorPtr_t proj = c->configProjector ();
        if (c->apply (q)) {
          return true;
        }
        if (proj) {
          ::hpp::statistics::SuccessStatistics& ss = proj->statistics ();
          if (ss.nbFailure () > ss.nbSuccess ()) {
            hppDout (warning, c->name () << " fails often." << std::endl << ss);
          } else {
            hppDout (warning, c->name () << " succeeds at rate " << (double)(ss.nbSuccess ()) / ss.numberOfObservations () << ".");
          }
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
        config_ = q2;
        if (!waypoint_.first->applyConstraints (q1, config_))
          return false;
        if (!waypoint_.first->build (pathToWaypoint, q1, config_, d))
          return false;
        core::PathVectorPtr_t pv = HPP_DYNAMIC_PTR_CAST (core::PathVector, pathToWaypoint);
        if (!pv) {
          pv = core::PathVector::create (graph_.lock ()->robot ()->configSize ());
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
        if (!waypoint_.first->applyConstraints (qoffset, q))
          return false;
        return Edge::applyConstraints (qoffset, q);
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
          edge->isInNodeFrom (isInNodeFrom ());
          we->createWaypoint (d-1, bname);
          edge = we;
        }
        ss.str (std::string ()); ss.clear ();
        ss << bname << "_e" << d;
        edge->name (ss.str ());
        waypoint_ = Waypoint (edge, node);
        config_ = Configuration_t(graph_.lock ()->robot ()->configSize ());
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

      std::ostream& WaypointEdge::dotPrint (std::ostream& os) const
      {
        os << from()->id () << " -> " << to()->id () << " [shape=onormal,label=\"" << name () << "\"];";
        return os;
      }

      std::ostream& LevelSetEdge::print (std::ostream& os) const
      {
        os << "|   |   |-- ";
        GraphComponent::print (os)
          << " (level set) --> " << to ()->name ();
        return os;
      }

      std::ostream& LevelSetEdge::dotPrint (std::ostream& os) const
      {
        os << from()->id () << " -> " << to()->id () << " [shape=onormal,label=\"" << name () << "\"];";
        return os;
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
        cs->offsetFromConfig (q_offset);

        const ConfigProjectorPtr_t cp = cs->configProjector ();
        if (cp) {
          vector_t offset = cp->offsetFromConfig (q_offset);
          size_t row = 0, nbRows = 0;
          for (DifferentiableFunctions_t::const_iterator it = extraNumericalFunctions_.begin ();
              it != extraNumericalFunctions_.end (); ++it) {
            const core::DifferentiableFunction& f = *(it->first);
            nbRows = f.outputSize ();
            vector_t value = vector_t::Zero (nbRows);
            if (f.isParametric ()) {
              f (value, levelsetTarget);
            }
            offset.segment (row, nbRows) = value;
            row += nbRows;
          }
          cp->offset (offset);
        }
        for (LockedDofs_t::const_iterator it = extraLockedDofs_.begin ();
            it != extraLockedDofs_.end (); ++it) {
          (*it)->offsetFromConfig (levelsetTarget);
        }

        // Eventually, do the projection.
        if (cs->apply (q))
          return true;
        if (cp) {
          ::hpp::statistics::SuccessStatistics& ss = cp->statistics ();
          if (ss.nbFailure () > ss.nbSuccess ()) {
            hppDout (warning, cs->name () << " fails often." << std::endl << ss);
          } else {
            hppDout (warning, cs->name () << " succeeds at rate " << (double)(ss.nbSuccess ()) / ss.numberOfObservations () << ".");
          }
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

        if (!extraNumericalFunctions_.empty ()) {
          ConfigProjectorPtr_t proj = ConfigProjector::create(g->robot(), "proj_" + n, g->errorThreshold(), g->maxIterations());
          for (DifferentiableFunctions_t::const_iterator it = extraNumericalFunctions_.begin ();
              it != extraNumericalFunctions_.end (); ++it) {
            proj->addConstraint (it->first);
          }
          constraint->addConstraint (proj);
        }

        for (LockedDofs_t::const_iterator it = extraLockedDofs_.begin ();
            it != extraLockedDofs_.end (); ++it)
          constraint->addConstraint (*it);

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
          bool gHasDiffFunc = g->insertNumericalConstraints (proj);
          for (DifferentiableFunctions_t::const_iterator it = extraNumericalFunctions_.begin ();
              it != extraNumericalFunctions_.end (); ++it) {
            proj->addConstraint (it->first, it->second);
          }
          if (gHasDiffFunc | !extraNumericalFunctions_.empty ()
              | insertNumericalConstraints (proj)
              | to ()->insertNumericalConstraints (proj))
            constraint->addConstraint (proj);

          g->insertLockedDofs (constraint);
          for (LockedDofs_t::const_iterator it = extraLockedDofs_.begin ();
              it != extraLockedDofs_.end (); ++it) {
            constraint->addConstraint (*it);
          }
          insertLockedDofs (constraint);
          to ()->insertLockedDofs (constraint);
          extraConstraints_->set (constraint);
        }
        return extraConstraints_->get ();
      }

      void LevelSetEdge::insertConfigConstraint (const DifferentiableFunctionPtr_t function, const EquationTypePtr_t ineq)
      {
        extraNumericalFunctions_.push_back (DiffFuncAndIneqPair_t (function, ineq));
      }

      void LevelSetEdge::insertConfigConstraint (const LockedDofPtr_t lockedDof)
      {
        extraLockedDofs_.push_back (lockedDof);
      }

      LevelSetEdge::LevelSetEdge (): extraConstraints_ (new Constraint_t()) {}

      LevelSetEdge::~LevelSetEdge ()
      {
        if (extraConstraints_  ) delete extraConstraints_;
      }
    } // namespace graph
  } // namespace manipulation
} // namespace hpp
