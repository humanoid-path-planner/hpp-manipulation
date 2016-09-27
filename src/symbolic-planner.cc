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

#include "hpp/manipulation/symbolic-planner.hh"

#include <boost/tuple/tuple.hpp>
#include <boost/assign/list_of.hpp>

#include <hpp/util/pointer.hh>
#include <hpp/util/timer.hh>
#include <hpp/util/assertion.hh>

#include <hpp/pinocchio/configuration.hh>
#include <hpp/pinocchio/collision-object.hh>

#include <hpp/core/roadmap.hh>
#include <hpp/core/distance.hh>
#include <hpp/core/path-validation.hh>
#include <hpp/core/connected-component.hh>
#include <hpp/core/path-projector.hh>
#include <hpp/core/projection-error.hh>
#include <hpp/core/nearest-neighbor.hh>
#include <hpp/core/basic-configuration-shooter.hh>
#include <hpp/core/path-validation-report.hh>
#include <hpp/core/collision-validation-report.hh>

#include "hpp/manipulation/graph/statistics.hh"
#include "hpp/manipulation/device.hh"
#include "hpp/manipulation/problem.hh"
#include "hpp/manipulation/roadmap.hh"
#include "hpp/manipulation/roadmap-node.hh"
#include "hpp/manipulation/symbolic-component.hh"
#include "hpp/manipulation/graph-path-validation.hh"
#include "hpp/manipulation/graph/edge.hh"
#include "hpp/manipulation/graph/graph.hh"
#include "hpp/manipulation/graph/state-selector.hh"

#define CastToWSC_ptr(var, scPtr) \
  WeighedSymbolicComponentPtr_t var = \
  HPP_DYNAMIC_PTR_CAST(WeighedSymbolicComponent,scPtr)

namespace hpp {
  namespace manipulation {
    using core::CollisionValidationReport;
    using core::CollisionValidationReportPtr_t;
    using core::CollisionObjectConstPtr_t;
    typedef graph::Edge::RelativeMotion RelativeMotion;

    namespace {
      HPP_DEFINE_TIMECOUNTER(oneStep);
      HPP_DEFINE_TIMECOUNTER(extend);
      HPP_DEFINE_TIMECOUNTER(tryConnect);
      HPP_DEFINE_TIMECOUNTER(nearestNeighbor);
      HPP_DEFINE_TIMECOUNTER(delayedEdges);
      HPP_DEFINE_TIMECOUNTER(tryConnectNewNodes);
      HPP_DEFINE_TIMECOUNTER(tryConnectToRoadmap);
      /// extend steps
      HPP_DEFINE_TIMECOUNTER(chooseEdge);
      HPP_DEFINE_TIMECOUNTER(applyConstraints);
      HPP_DEFINE_TIMECOUNTER(buildPath);
      HPP_DEFINE_TIMECOUNTER(projectPath);
      HPP_DEFINE_TIMECOUNTER(validatePath);

      struct WeighedSymbolicComponentComp {
        bool operator() (const SymbolicComponentPtr_t& lhs, const SymbolicComponentPtr_t& rhs)
        {
          return HPP_DYNAMIC_PTR_CAST(WeighedSymbolicComponent, lhs)->weight_
            > HPP_DYNAMIC_PTR_CAST(WeighedSymbolicComponent, rhs)->weight_;
        }
      };
      typedef std::list<WeighedSymbolicComponentPtr_t> SymbolicComponentList_t;
      SymbolicComponentList_t sorted_list (const SymbolicComponents_t& sc)
      {
        SymbolicComponentList_t l;
        WeighedSymbolicComponentComp comp;
        for (SymbolicComponents_t::const_iterator _sc = sc.begin();
            _sc != sc.end(); ++_sc)
          l.insert (std::upper_bound(l.begin(), l.end(), *_sc, comp),
              HPP_DYNAMIC_PTR_CAST(WeighedSymbolicComponent, *_sc));
        return l;
      }

      struct DistanceToConfiguration {
        const Configuration_t& q_;
        core::Distance& d_;
        value_type mind_;
        RoadmapNodePtr_t node_;
        DistanceToConfiguration(const Configuration_t& q, core::Distance& d)
          : q_(q), d_(d), mind_(std::numeric_limits<value_type>::max()) {}
        void operator() (const RoadmapNodePtr_t& n)
        {
          value_type dist = d_ (*(n->configuration()), q_);
          if (dist < mind_) {
            node_ = n;
            mind_ = dist;
          }
        }
      };

      bool belongs (const ConfigurationPtr_t& q, const core::Nodes_t& nodes)
      {
        for (core::Nodes_t::const_iterator itNode = nodes.begin ();
            itNode != nodes.end (); ++itNode) {
          if (*((*itNode)->configuration ()) == *q) return true;
        }
        return false;
      }
    }

    const std::vector<SymbolicPlanner::Reason>
      SymbolicPlanner::reasons_ = boost::assign::list_of
      (SuccessBin::createReason ("[Fail] Projection"))
      (SuccessBin::createReason ("[Fail] SteeringMethod"))
      (SuccessBin::createReason ("[Fail] Path validation returned length 0"))
      (SuccessBin::createReason ("[Fail] Path could not be projected"))
      (SuccessBin::createReason ("[Fail] Unknow"))
      (SuccessBin::createReason ("[Info] Path could not be fully projected"))
      (SuccessBin::createReason ("[Info] Path could not be fully validated"))
      (SuccessBin::createReason ("[Info] Extended partly"));

    SymbolicPlannerPtr_t SymbolicPlanner::create (const core::Problem& problem,
        const core::RoadmapPtr_t& roadmap)
    {
      SymbolicPlanner* ptr;
      core::RoadmapPtr_t r2 = roadmap;
      RoadmapPtr_t r;
      try {
        const Problem& p = dynamic_cast < const Problem& > (problem);
        RoadmapPtr_t r = HPP_DYNAMIC_PTR_CAST (Roadmap, r2);
        ptr = new SymbolicPlanner (p, r);
      } catch (std::exception&) {
        if (!r)
          throw std::invalid_argument ("The roadmap must be of type hpp::manipulation::Roadmap.");
        else
        throw std::invalid_argument ("The problem must be of type hpp::manipulation::Problem.");
      }
      SymbolicPlannerPtr_t shPtr (ptr);
      ptr->init (shPtr);
      return shPtr;
    }

    SymbolicPlanner::ErrorFreqs_t SymbolicPlanner::getEdgeStat
      (const graph::EdgePtr_t& edge) const
    {
      const std::size_t& id = edge->id ();
      ErrorFreqs_t ret;
      if (indexPerEdgeStatistics_.size() <= id ||
         indexPerEdgeStatistics_[id] < 0) {
        for (std::size_t i = 0; i < reasons_.size(); ++i) ret.push_back (0);
      } else {
        const SuccessStatistics& ss =
          perEdgeStatistics_[indexPerEdgeStatistics_[id]];
        ret.push_back (ss.nbSuccess ());
        for (std::size_t i = 0; i < reasons_.size(); ++i)
          ret.push_back (ss.nbFailure (reasons_[i]));
      }
      return ret;
    }

    StringList_t SymbolicPlanner::errorList ()
    {
      StringList_t ret;
      ret.push_back ("Success");
      for (std::size_t i = 0; i < reasons_.size(); ++i) ret.push_back (reasons_[i].what);
      return ret;
    }

    void SymbolicPlanner::oneStep ()
    {
      HPP_START_TIMECOUNTER(oneStep);

      // Get the robot
      DevicePtr_t robot = HPP_DYNAMIC_PTR_CAST(Device, problem ().robot ());
      HPP_ASSERT(robot);
      // Get the roadmap and the symbolic components
      RoadmapPtr_t rdm = HPP_DYNAMIC_PTR_CAST(Roadmap, roadmap());
      HPP_ASSERT(rdm);
      SymbolicComponentList_t scs = sorted_list (rdm->symbolicComponents());

      core::Nodes_t newNodes;
      core::PathPtr_t path;

      typedef boost::tuple <RoadmapNodePtr_t, ConfigurationPtr_t, core::PathPtr_t,
              ExtendStatus>
	DelayedEdge_t;
      typedef std::vector <DelayedEdge_t> DelayedEdges_t;
      DelayedEdges_t delayedEdges;

      // Pick a random node
      ConfigurationPtr_t q_rand = shooter_->shoot();

      // Extend each connected component
      for (core::ConnectedComponents_t::const_iterator itcc =
          rdm->connectedComponents ().begin ();
          itcc != rdm->connectedComponents ().end (); ++itcc) {
        // Find the symbolic component in this connected component which has
        // the highest value.
        SymbolicComponentPtr_t sc;
        for (SymbolicComponentList_t::iterator _sc = scs.begin();
            _sc != scs.end(); ++_sc) {
          sc = *_sc;
          if (sc->connectedComponent() == *itcc) break;
        }
        if (!sc) {
          hppDout (error, "There should always be a connected component corresponding to symbolic component");
          continue;
        }

        // Find the nearest neighbor.
        HPP_START_TIMECOUNTER(nearestNeighbor);
        DistanceToConfiguration dtc (*q_rand, *rdm->distance());
        RoadmapNodePtr_t near =
          std::for_each(sc->nodes().begin(), sc->nodes().end(), dtc).node_;
        HPP_STOP_TIMECOUNTER(nearestNeighbor);
        HPP_DISPLAY_LAST_TIMECOUNTER(nearestNeighbor);
        if (!near) continue;

        // Extension
        HPP_START_TIMECOUNTER(extend);
        ExtendStatus status;
        bool pathIsValid = extend (near, q_rand, path, status);
        HPP_STOP_TIMECOUNTER(extend);
        HPP_DISPLAY_LAST_TIMECOUNTER(extend);

        // Insert new path to q_near in roadmap
        if (pathIsValid) {
          value_type t_final = path->timeRange ().second;
          if (t_final != path->timeRange ().first) {
            bool success;
            ConfigurationPtr_t q_new (new Configuration_t
                ((*path) (t_final, success)));
            delayedEdges.push_back (DelayedEdge_t (near, q_new, path, status));
          }
        } else {
          updateWeightsAndProbabilities (near, RoadmapNodePtr_t(), status);
        }
      }

      // Insert delayed edges
      HPP_START_TIMECOUNTER(delayedEdges);
      for (DelayedEdges_t::const_iterator itEdge = delayedEdges.begin ();
	   itEdge != delayedEdges.end (); ++itEdge) {
	const core::NodePtr_t& near = itEdge-> get <0> ();
	const ConfigurationPtr_t& q_new = itEdge-> get <1> ();
	const core::PathPtr_t& validPath = itEdge-> get <2> ();
        core::NodePtr_t newNode = rdm->addNode (q_new);
	roadmap()->addEdge (near, newNode, validPath);
        core::interval_t timeRange = validPath->timeRange ();
	roadmap()->addEdge (newNode, near, validPath->extract
			     (core::interval_t (timeRange.second ,
                                                timeRange.first)));
        updateWeightsAndProbabilities (
            static_cast<RoadmapNode*>(near),
            static_cast<RoadmapNode*>(newNode),
            itEdge->get<3>());
      }
      HPP_STOP_TIMECOUNTER(delayedEdges);

      // Try to connect the new nodes together
      HPP_START_TIMECOUNTER(tryConnectNewNodes);
      const std::size_t nbConn = tryConnectNewNodes (newNodes);
      HPP_STOP_TIMECOUNTER(tryConnectNewNodes);
      HPP_DISPLAY_LAST_TIMECOUNTER(tryConnectNewNodes);
      if (nbConn == 0) {
        HPP_START_TIMECOUNTER(tryConnectToRoadmap);
        tryConnectToRoadmap (newNodes);
        HPP_STOP_TIMECOUNTER(tryConnectToRoadmap);
        HPP_DISPLAY_LAST_TIMECOUNTER(tryConnectToRoadmap);
      }
      HPP_STOP_TIMECOUNTER(oneStep);
      HPP_DISPLAY_LAST_TIMECOUNTER(oneStep);
      HPP_DISPLAY_TIMECOUNTER(oneStep);
      HPP_DISPLAY_TIMECOUNTER(extend);
      HPP_DISPLAY_TIMECOUNTER(tryConnect);
      HPP_DISPLAY_TIMECOUNTER(tryConnectNewNodes);
      HPP_DISPLAY_TIMECOUNTER(tryConnectToRoadmap);
      HPP_DISPLAY_TIMECOUNTER(nearestNeighbor);
      HPP_DISPLAY_TIMECOUNTER(delayedEdges);
      HPP_DISPLAY_TIMECOUNTER(chooseEdge);
      HPP_DISPLAY_TIMECOUNTER(applyConstraints);
      HPP_DISPLAY_TIMECOUNTER(buildPath);
      HPP_DISPLAY_TIMECOUNTER(projectPath);
      HPP_DISPLAY_TIMECOUNTER(validatePath);
    }

    bool SymbolicPlanner::extend(
        RoadmapNodePtr_t n_near,
        const ConfigurationPtr_t& q_rand,
        core::PathPtr_t& validPath,
        ExtendStatus& status)
    {
      status.info = SUCCESS;
      graph::GraphPtr_t graph = problem_.constraintGraph ();
      // Select next node in the constraint graph.
      const ConfigurationPtr_t q_near = n_near->configuration ();

      HPP_START_TIMECOUNTER (chooseEdge);
      // This code should go into a NodeSelector derived class.
      WeighedSymbolicComponentPtr_t wscPtr = HPP_DYNAMIC_PTR_CAST
        (WeighedSymbolicComponent, n_near->symbolicComponent());
      if (wscPtr) {
        WeighedSymbolicComponent wsc = *wscPtr;
        value_type R = rand() / RAND_MAX;
        std::size_t i = 0;
        for (value_type sum = wsc.p_[0]; sum < R; sum += wsc.p_[i]) { ++i; }
        assert(i < wsc.edges_.size());
        status.edge = wsc.edges_[i];
      } else status.edge = graph->chooseEdge (n_near);
      HPP_STOP_TIMECOUNTER (chooseEdge);
      if (!status.edge) {
        status.status = UNKNOWN;
        return false;
      }

      qProj_ = *q_rand;
      HPP_START_TIMECOUNTER (applyConstraints);
      SuccessStatistics& es = edgeStat (status.edge);
      if (!status.edge->applyConstraints (n_near, qProj_)) {
        HPP_STOP_TIMECOUNTER (applyConstraints);
        es.addFailure (reasons_[PROJECTION]);
        status.status = PROJECTION;
        return false;
      }
      HPP_STOP_TIMECOUNTER (applyConstraints);

      core::PathPtr_t path;
      HPP_START_TIMECOUNTER (buildPath);
      if (!status.edge->build (path, *q_near, qProj_)) {
        HPP_STOP_TIMECOUNTER (buildPath);
        es.addFailure (reasons_[STEERING_METHOD]);
        status.status = STEERING_METHOD;
        return false;
      }
      HPP_STOP_TIMECOUNTER (buildPath);

      core::PathPtr_t projPath;
      bool projShorter = false;
      PathProjectorPtr_t pathProjector = problem_.pathProjector ();
      if (pathProjector) {
        HPP_START_TIMECOUNTER (projectPath);
        projShorter = !pathProjector->apply (path, projPath);
        if (projShorter) {
          if (!projPath || projPath->length () == 0) {
            HPP_STOP_TIMECOUNTER (projectPath);
            es.addFailure (reasons_[PATH_PROJECTION_ZERO]);
            status.status = PATH_PROJECTION_ZERO;
            return false;
          }
          status.info = PATH_PROJECTION_SHORTER;
          es.addFailure (reasons_[PATH_PROJECTION_SHORTER]);
        }
        HPP_STOP_TIMECOUNTER (projectPath);
      } else projPath = path;
      GraphPathValidationPtr_t pathValidation (problem_.pathValidation ());
      core::PathPtr_t fullValidPath;

      HPP_START_TIMECOUNTER (validatePath);
      bool fullyValid = false;
      try {
        fullyValid = pathValidation->validate
          (projPath, false, fullValidPath, status.validationReport);
      } catch (const core::projection_error& e) {
        hppDout (error, e.what ());
        es.addFailure (reasons_[PATH_VALIDATION_ZERO]);
        status.status = PATH_VALIDATION_ZERO;
        return false;
      }
      HPP_STOP_TIMECOUNTER (validatePath);

      if (fullValidPath->length () == 0) {
        es.addFailure (reasons_[PATH_VALIDATION_ZERO]);
        validPath = fullValidPath;
        status.status = PATH_PROJECTION_ZERO;
        return false;
      } else {
        if (!fullyValid) {
          es.addFailure (reasons_[PATH_VALIDATION_SHORTER]);
          status.status = (projShorter?PATH_PROJECTION_AND_VALIDATION_SHORTER:PATH_VALIDATION_SHORTER);
        }
        if (extendStep_ == 1 || fullyValid) {
          validPath = fullValidPath;
        } else {
          const value_type& length = fullValidPath->length();
          const value_type& t_init = fullValidPath->timeRange ().first;
          try {
            validPath = fullValidPath->extract
              (core::interval_t(t_init, t_init + length * extendStep_));
          } catch (const core::projection_error& e) {
            hppDout (error, e.what());
            es.addFailure (reasons_[PATH_PROJECTION_SHORTER]);
            status.status = UNKNOWN;
            return false;
          }
        }
        hppDout (info, "Extension:" << std::endl
            << es);
      }
      if (!projShorter && fullValidPath) es.addSuccess ();
      else es.addFailure (reasons_[PARTLY_EXTENDED]);
      status.status = SUCCESS;
      return true;
    }

    SymbolicPlanner::SuccessStatistics& SymbolicPlanner::edgeStat
      (const graph::EdgePtr_t& edge)
    {
      const std::size_t& id = edge->id ();
      if (indexPerEdgeStatistics_.size () <= id) {
        indexPerEdgeStatistics_.resize (id + 1, -1);
      }
      if (indexPerEdgeStatistics_[id] < 0) {
        indexPerEdgeStatistics_[id] = (size_type)perEdgeStatistics_.size();
        perEdgeStatistics_.push_back (SuccessStatistics (edge->name (), 2));
      }
      return perEdgeStatistics_[indexPerEdgeStatistics_[id]];
    }

    inline std::size_t SymbolicPlanner::tryConnectToRoadmap (const core::Nodes_t nodes)
    {
      const core::SteeringMethodPtr_t& sm (problem ().steeringMethod ());
      core::PathValidationPtr_t pathValidation (problem ().pathValidation ());
      PathProjectorPtr_t pathProjector (problem().pathProjector ());
      core::PathPtr_t path, projPath, validPath;
      graph::GraphPtr_t graph = problem_.constraintGraph ();
      bool connectSucceed = false;
      std::size_t nbConnection = 0;
      const std::size_t K = 7;
      value_type distance;
      for (core::Nodes_t::const_iterator itn1 = nodes.begin ();
          itn1 != nodes.end (); ++itn1) {
        ConfigurationPtr_t q1 ((*itn1)->configuration ());
        connectSucceed = false;
        for (core::ConnectedComponents_t::const_iterator itcc =
            roadmap ()->connectedComponents ().begin ();
            itcc != roadmap ()->connectedComponents ().end (); ++itcc) {
          if (*itcc == (*itn1)->connectedComponent ())
            continue;
          core::Nodes_t knearest = roadmap()->nearestNeighbor ()
            ->KnearestSearch (q1, *itcc, K, distance);
          for (core::Nodes_t::const_iterator itn2 = knearest.begin ();
              itn2 != knearest.end (); ++itn2) {
            bool _1to2 = (*itn1)->isOutNeighbor (*itn2);
            bool _2to1 = (*itn1)->isInNeighbor (*itn2);
            if (_1to2 && _2to1) {
              hppDout (info, "the two nodes are already connected");
              continue;
            }
            ConfigurationPtr_t q2 ((*itn2)->configuration ());
            assert (*q1 != *q2);
            path = (*sm) (*q1, *q2);
            if (!path) continue;
            if (pathProjector) {
              if (!pathProjector->apply (path, projPath)) continue;
            } else projPath = path;
	    PathValidationReportPtr_t report;
            if (pathValidation->validate (projPath, false, validPath, report)) {
              nbConnection++;
              if (!_1to2) roadmap ()->addEdge (*itn1, *itn2, projPath);
              if (!_2to1) {
                core::interval_t timeRange = projPath->timeRange ();
                roadmap ()->addEdge (*itn2, *itn1, projPath->extract
                    (core::interval_t (timeRange.second,
                                       timeRange.first)));
              }
              connectSucceed = true;
              break;
            }
          }
          if (connectSucceed) break;
        }
      }
      return nbConnection;
    }

    inline std::size_t SymbolicPlanner::tryConnectNewNodes (const core::Nodes_t nodes)
    {
      const core::SteeringMethodPtr_t& sm (problem ().steeringMethod ());
      core::PathValidationPtr_t pathValidation (problem ().pathValidation ());
      PathProjectorPtr_t pathProjector (problem().pathProjector ());
      core::PathPtr_t path, projPath, validPath;
      graph::GraphPtr_t graph = problem_.constraintGraph ();
      std::size_t nbConnection = 0;
      for (core::Nodes_t::const_iterator itn1 = nodes.begin ();
          itn1 != nodes.end (); ++itn1) {
        ConfigurationPtr_t q1 ((*itn1)->configuration ());
        for (core::Nodes_t::const_iterator itn2 = boost::next (itn1);
            itn2 != nodes.end (); ++itn2) {
          if ((*itn1)->connectedComponent () == (*itn2)->connectedComponent ())
            continue;
          bool _1to2 = (*itn1)->isOutNeighbor (*itn2);
          bool _2to1 = (*itn1)->isInNeighbor (*itn2);
          if (_1to2 && _2to1) {
            hppDout (info, "the two nodes are already connected");
            continue;
          }
          ConfigurationPtr_t q2 ((*itn2)->configuration ());
          assert (*q1 != *q2);
          path = (*sm) (*q1, *q2);
          if (!path) continue;
          if (pathProjector) {
            if (!pathProjector->apply (path, projPath)) continue;
          } else projPath = path;
          PathValidationReportPtr_t report;
          if (pathValidation->validate (projPath, false, validPath, report)) {
            nbConnection++;
            if (!_1to2) roadmap ()->addEdge (*itn1, *itn2, projPath);
            if (!_2to1) {
              core::interval_t timeRange = projPath->timeRange ();
              roadmap ()->addEdge (*itn2, *itn1, projPath->extract
                  (core::interval_t (timeRange.second,
                                     timeRange.first)));
            }
          }
        }
      }
      return nbConnection;
    }

    inline void SymbolicPlanner::updateWeightsAndProbabilities (
        const RoadmapNodePtr_t& near, const RoadmapNodePtr_t& newN,
        const ExtendStatus& extend)
    {
      // 0.99 ^ 26 * 1.3 ~= 1  => A new extension followed by 26 failures
      // results in similar weights for the symbolic components.
      const value_type weightInc = 1.3;
      const value_type weightDec = 0.99;
      CastToWSC_ptr (oldWSC, near->symbolicComponent());
      CollisionValidationReportPtr_t colRep;
      switch (extend.status) {
        case SUCCESS:
          {
            CastToWSC_ptr (newWSC, newN->symbolicComponent());
            switch (extend.info) {
              case SUCCESS:
                // If the corresponding edge is a loop, no adjustment.
                // Otherwise, a new symbolic component has been created. This new SC
                // should have higher priority than the old one.
                if (oldWSC != newWSC) {
                  newWSC->weight_ = oldWSC->weight_ * weightInc;
                  // Maybe we should decrease the edge probability as well.
                  // A new connection with this edge should be tried whenever
                  // we have tried to extend the new SC and we could not.
                }
                oldWSC->weight_ *= weightDec;
                updateEdgeProba(extend.edge, oldWSC, 0.99); // Rate of change should not be too big.
                break;
              case PATH_PROJECTION_SHORTER:
              case PATH_PROJECTION_AND_VALIDATION_SHORTER:
                // Hard to say what to do on this case...
                break;
              case PATH_VALIDATION_SHORTER:
                // The path was made shorter, check the validation report to know
                // why.
                colRep = HPP_DYNAMIC_PTR_CAST(CollisionValidationReport,
                    extend.validationReport->configurationReport);
                if (colRep) {
                  CollisionObjectConstPtr_t o1 = colRep->object1, o2 = colRep->object2;
                  // If it is a collision between the robot and an unactuated object,
                  // which cannot move in this state.
                  if (o1->joint() != NULL && o2->joint() != NULL) {
                    // TODO: Currently, the RelativeMotion matrix does not cover cases
                    // like ConvexShapeContact (2 function making a 6D constraints).
                    RelativeMotion::matrix_type m = extend.edge->relativeMotion();
                    size_type i0 = RelativeMotion::idx(JointPtr_t());
                    size_type i1 = RelativeMotion::idx(o1->joint());
                    size_type i2 = RelativeMotion::idx(o2->joint());
                    if (m(i0, i1) == RelativeMotion::Parameterized
                        || m(i0, i1) == RelativeMotion::Constrained) {
                      // Object1 should be moved out of the way first...
                      updateEdgeProba(extend.edge, oldWSC, 0.99); // Rate of change should not be too big.
                    }
                    if (m(i0, i2) == RelativeMotion::Parameterized
                        || m(i0, i2) == RelativeMotion::Constrained) {
                      // Object2 should be moved out of the way first...
                      updateEdgeProba(extend.edge, oldWSC, 0.99); // Rate of change should not be too big.
                    }
                  }
                }
                break;
            }
          }
          break;
          // For cases below, newN is a NULL pointer.
        case PROJECTION:
        case PATH_PROJECTION_ZERO: // unclear for PATH_PROJECTION_ZERO
          // It was not possible to project. It may be that projection is not
          // possible, or only that it is not yet possible.
          // We decrease the probability of sampling this edge.
          updateEdgeProba(extend.edge, oldWSC, 0.99); // Rate of change should not be too big.
          oldWSC->weight_ *= weightDec;
          break;
        case PATH_VALIDATION_ZERO:
          // The path validation report will say what's wrong.
          // The only interesting thing are collisions with objects.
          colRep = HPP_DYNAMIC_PTR_CAST(CollisionValidationReport,
                extend.validationReport->configurationReport);
          if (colRep) {
            CollisionObjectConstPtr_t o1 = colRep->object1, o2 = colRep->object2;
            // If it is a collision between the robot and an unactuated object,
            // which cannot move in this state.
            if (o1->joint() != NULL && o2->joint() != NULL) {
              // TODO: Currently, the RelativeMotion matrix does not cover cases
              // like ConvexShapeContact (2 function making a 6D constraints).
              RelativeMotion::matrix_type m = extend.edge->relativeMotion();
              size_type i0 = RelativeMotion::idx(JointPtr_t());
              size_type i1 = RelativeMotion::idx(o1->joint());
              size_type i2 = RelativeMotion::idx(o2->joint());
              if (m(i0, i1) == RelativeMotion::Parameterized
                  || m(i0, i1) == RelativeMotion::Constrained) {
                // Object1 should be moved out of the way first...
                updateEdgeProba(extend.edge, oldWSC, 0.99); // Rate of change should not be too big.
              }
              if (m(i0, i2) == RelativeMotion::Parameterized
                  || m(i0, i2) == RelativeMotion::Constrained) {
                // Object2 should be moved out of the way first...
                updateEdgeProba(extend.edge, oldWSC, 0.99); // Rate of change should not be too big.
              }
            }
          }
          break;
        case UNKNOWN:
        case STEERING_METHOD:
          // Hard to say what to do.
          oldWSC->weight_ *= weightDec;
          break;
      }
    }

    void SymbolicPlanner::updateEdgeProba (
        const graph::EdgePtr_t edge,
        WeighedSymbolicComponentPtr_t wsc,
        const value_type alpha) {
      size_type i = wsc->indexOf (edge);
      assert(i < wsc->p_.size());
      const value_type pi = wsc->p_[i];
      wsc->p_ *= ( 1 - alpha * pi ) / ( 1 - pi );
      wsc->p_[i] = alpha * pi;
      wsc->normalizeProba();
    }

    SymbolicPlanner::SymbolicPlanner (const Problem& problem,
        const RoadmapPtr_t& roadmap) :
      core::PathPlanner (problem, roadmap),
      shooter_ (problem.configurationShooter()),
      problem_ (problem), roadmap_ (roadmap),
      extendStep_ (1),
      qProj_ (problem.robot ()->configSize ())
    {}

    void SymbolicPlanner::init (const SymbolicPlannerWkPtr_t& weak)
    {
      core::PathPlanner::init (weak);
      weakPtr_ = weak;
    }
  } // namespace manipulation
} // namespace hpp
