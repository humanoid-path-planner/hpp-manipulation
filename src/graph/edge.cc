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
#include <hpp/core/path-vector.hh>

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
        os << "|   |   |-- " << (GraphComponent*) this
          << " --> " << to_.lock ()->name () << std::endl;
        return os;
      }

      ConstraintSetPtr_t Edge::configConstraint() const
      {
        if (!*configConstraints_) {
          std::string n = "(" + name () + ")";
          GraphPtr_t g = graph_.lock ();

          ConstraintSetPtr_t constraint = ConstraintSet::create (g->robot (), "Set " + n);

          ConfigProjectorPtr_t proj = ConfigProjector::create(g->robot(), "proj_" + n, g->errorThreshold(), g->maxIterations());
          g->insertNumericalConstraints (proj);
          insertNumericalConstraints (proj);
          to ()->insertNumericalConstraints (proj);
          constraint->addConstraint (HPP_DYNAMIC_PTR_CAST(Constraint, proj));

          g->insertLockedDofs (constraint);
          insertLockedDofs (constraint);
          to ()->insertLockedDofs (constraint);
          configConstraints_->set (constraint);
        }
        return configConstraints_->get ();
      }

      ConstraintSetPtr_t Edge::pathConstraint() const
      {
        if (!*pathConstraints_) {
          std::string n = "(" + name () + ")";
          GraphPtr_t g = graph_.lock ();

          ConstraintSetPtr_t constraint = ConstraintSet::create (g->robot (), "Set " + n);

          ConfigProjectorPtr_t proj = ConfigProjector::create(g->robot(), "proj_" + n, g->errorThreshold(), g->maxIterations());
          g->insertNumericalConstraints (proj);
          insertNumericalConstraints (proj);
          node ()->insertNumericalConstraintsForPath (proj);
          constraint->addConstraint (HPP_DYNAMIC_PTR_CAST(Constraint, proj));

          g->insertLockedDofs (constraint);
          insertLockedDofs (constraint);
          node ()->insertLockedDofs (constraint);
          pathConstraints_->set (constraint);
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

      bool Edge::applyConstraints (core::NodePtr_t nnear, ConfigurationOut_t q) const
      {
        return applyConstraints (*(nnear->configuration ()), q);
      }

      bool Edge::applyConstraints (ConfigurationIn_t qoffset, ConfigurationOut_t q) const
      {
        configConstraint ()->offsetFromConfig (qoffset);
        if (configConstraint ()->apply (q))
          return true;
        typedef ::hpp::statistics::SuccessStatistics SuccessStatistics;
        SuccessStatistics& ss = configConstraint ()->configProjector ()->statistics ();
        if (ss.nbFailure () > ss.nbSuccess ()) {
          hppDout (warning, configConstraint ()->name () << " fails often." << std::endl << ss);
        } else {
          hppDout (warning, configConstraint ()->name () << " succeeds at rate " << (double)(ss.nbSuccess ()) / ss.numberOfObservations () << ".");
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
        createWaypoint ();
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
        core::PathVectorPtr_t pv = core::PathVector::create (graph_.lock ()->robot ()->configSize ());
        path = pv;
        pv->appendPath (pathToWaypoint);

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

      void WaypointEdge::createWaypoint ()
      {
        NodePtr_t node = Node::create ();
        node->parentGraph(graph_);
        EdgePtr_t edge = Edge::create (graph_, from (), node);
        edge->isInNodeFrom (isInNodeFrom ());
        waypoint_ = Waypoint (edge, node);
        config_ = Configuration_t(graph_.lock ()->robot ()->configSize ());
      }

      EdgePtr_t WaypointEdge::waypoint () const
      {
        return waypoint_.first;
      }

      std::ostream& WaypointEdge::print (std::ostream& os) const
      {
        os << "|   |   |-- " << (GraphComponent*) this
          << " (waypoint) --> " << to ()->name () << std::endl;
        return os;
      }

      std::ostream& LevelSetEdge::print (std::ostream& os) const
      {
        os << "|   |   |-- " << (GraphComponent*) this
          << " (waypoint) --> " << to ()->name () << std::endl;
        return os;
      }

      bool LevelSetEdge::applyConstraints (ConfigurationIn_t, ConfigurationOut_t) const
      {
        throw std::logic_error ("I need to know which connected component we wish to use.");
      }

      bool LevelSetEdge::applyConstraints (core::NodePtr_t n_offset, ConfigurationOut_t q) const
      {
        // First, get an offset from the histogram that is not in the same connected component.
        // Then, set the offset and do the actual projection.
        return Edge::applyConstraints (*(n_offset->configuration ()), q);
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

      void LevelSetEdge::histogram (LeafHistogramPtr_t hist)
      {
        hist_ = hist;
      }
    } // namespace graph
  } // namespace manipulation
} // namespace hpp
