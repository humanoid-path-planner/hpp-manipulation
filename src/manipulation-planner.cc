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

#include <hpp/util/assertion.hh>
#include <hpp/core/path-validation.hh>
#include <hpp/core/connected-component.hh>

#include "hpp/manipulation/graph/statistics.hh"
#include "hpp/manipulation/robot.hh"
#include "hpp/manipulation/problem.hh"
#include "hpp/manipulation/path-projector.hh"
#include "hpp/manipulation/path-projector/dichotomy.hh"
#include "hpp/manipulation/manipulation-planner.hh"
#include "hpp/manipulation/graph/edge.hh"

namespace hpp {
  namespace manipulation {
    ManipulationPlannerPtr_t ManipulationPlanner::create (const core::Problem& problem,
        const core::RoadmapPtr_t& roadmap)
    {
      ManipulationPlanner* ptr;
      try {
        const Problem& p = dynamic_cast < const Problem& > (problem);
        ptr = new ManipulationPlanner (p, roadmap);
      } catch (std::exception&) {
        throw std::invalid_argument ("The problem must be of type hpp::manipulation::Problem.");
      }
      ManipulationPlannerPtr_t shPtr (ptr);
      ptr->init (shPtr);
      return shPtr;
    }

    bool belongs (const ConfigurationPtr_t& q, const core::Nodes_t& nodes)
    {
      for (core::Nodes_t::const_iterator itNode = nodes.begin ();
          itNode != nodes.end (); ++itNode) {
        if (*((*itNode)->configuration ()) == *q) return true;
      }
      return false;
    }


    void ManipulationPlanner::oneStep ()
    {
      RobotPtr_t robot = HPP_DYNAMIC_PTR_CAST(Robot, problem ().robot ());
      HPP_ASSERT(robot);
      core::Nodes_t newNodes;
      core::PathPtr_t path;

      // Pick a random node
      ConfigurationPtr_t q_rand = shooter_->shoot();

      // Extend each connected component
      for (core::ConnectedComponents_t::const_iterator itcc =
          roadmap ()->connectedComponents ().begin ();
          itcc != roadmap ()->connectedComponents ().end (); ++itcc) {
        // Find the nearest neighbor.
        core::value_type distance;
        core::NodePtr_t near = roadmap ()->nearestNode (q_rand, *itcc, distance);

        bool pathIsValid = extend (near, q_rand, path);
        // Insert new path to q_near in roadmap
        if (pathIsValid) {
          value_type t_final = path->timeRange ().second;
          if (t_final != path->timeRange ().first) {
            ConfigurationPtr_t q_new (new Configuration_t
                ((*path) (t_final)));
            if (!belongs (q_new, newNodes)) {
              newNodes.push_back (roadmap ()->addNodeAndEdges
                  (near, q_new, path));
            } else {
              core::NodePtr_t newNode = roadmap ()->addNode (q_new);
              roadmap ()->addEdge (near, newNode, path);
              core::interval_t timeRange = path->timeRange ();
              roadmap ()->addEdge (newNode, near, path->extract
                  (core::interval_t (timeRange.second ,
                                     timeRange.first)));
            }
          }
        }
      }

      // Try to connect the new nodes together
      tryConnect (newNodes);
    }

    bool ManipulationPlanner::extend(
        const core::NodePtr_t& n_near,
        const ConfigurationPtr_t& q_rand,
        core::PathPtr_t& validPath)
    {
      graph::GraphPtr_t graph = problem_.constraintGraph ();
      // Select next node in the constraint graph.
      const ConfigurationPtr_t q_near = n_near->configuration ();
      graph::NodePtr_t node = graph->getNode (*q_near);
      graph::EdgePtr_t edge = graph->chooseEdge (node);
      qProj_ = *q_rand;
      if (!edge->applyConstraints (n_near, qProj_)) {
        addFailure (PROJECTION, edge);
        return false;
      }
      GraphSteeringMethodPtr_t sm = problem_.steeringMethod();
      core::PathPtr_t path;
      if (!edge->build (path, *q_near, qProj_, *(sm->distance ()))) {
        addFailure (STEERING_METHOD, edge);
        return false;
      }
      core::PathPtr_t projPath;
      if (!pathProjector_->apply (path, projPath)) {
        addFailure (PATH_PROJECTION, edge);
        if (!projPath || projPath->length () == 0) return false;
      }
      GraphPathValidationPtr_t pathValidation (problem_.pathValidation ());
      pathValidation->validate (path, false, validPath);
      if (validPath->length () == 0)
        addFailure (PATH_VALIDATION, edge);
      else {
        extendStatistics_.addSuccess ();
        hppDout (info, "Extension:" << std::endl
            << extendStatistics_);
        graph->getNode ((*validPath) (validPath->length ()));
      }
      return true;
    }

    void ManipulationPlanner::addFailure (TypeOfFailure t, const graph::EdgePtr_t& edge)
    {
      EdgeReasonMap::iterator it = failureReasons_.find (edge);
      if (it == failureReasons_.end ()) {
        std::string edgeStr = "(" + edge->name () + ")";
        Reasons r (SuccessBin::createReason ("Projection for " + edgeStr),
                   SuccessBin::createReason ("SteeringMethod for " + edgeStr),
                   SuccessBin::createReason ("PathValidation returned length 0 for " + edgeStr),
                   SuccessBin::createReason ("Path could not be fully projected for " + edgeStr));
        failureReasons_.insert (EdgeReasonPair (edge, r));
        extendStatistics_.addFailure (r.get (t));
        return;
      }
      Reasons r = it->second;
      extendStatistics_.addFailure (r.get (t));
      hppDout (info, "Extension failed." << std::endl
          << extendStatistics_);
    }

    inline void ManipulationPlanner::tryConnect (const core::Nodes_t nodes)
    {
      const core::SteeringMethodPtr_t& sm (problem ().steeringMethod ());
      core::PathValidationPtr_t pathValidation (problem ().pathValidation ());
      core::PathPtr_t path, validPath;
      graph::GraphPtr_t graph = problem_.constraintGraph ();
      bool connectSucceed = false;
      for (core::Nodes_t::const_iterator itn1 = nodes.begin ();
          itn1 != nodes.end (); ++itn1) {
        ConfigurationPtr_t q1 ((*itn1)->configuration ());
        connectSucceed = false;
        for (core::ConnectedComponents_t::const_iterator itcc =
            roadmap ()->connectedComponents ().begin ();
            itcc != roadmap ()->connectedComponents ().end (); ++itcc) {
          if (*itcc == (*itn1)->connectedComponent ())
            continue;
          for (core::Nodes_t::const_iterator itn2 = (*itcc)->nodes().begin ();
              itn2 != (*itcc)->nodes ().end (); ++itn2) {
            ConfigurationPtr_t q2 ((*itn2)->configuration ());
            assert (*q1 != *q2);
            path = (*sm) (*q1, *q2);
            if (path && pathValidation->validate (path, false, validPath)) {
              roadmap ()->addEdge (*itn1, *itn2, path);
              core::interval_t timeRange = path->timeRange ();
              roadmap ()->addEdge (*itn2, *itn1, path->extract
                  (core::interval_t (timeRange.second,
                                     timeRange.first)));
              connectSucceed = true;
              break;
            }
          }
          if (connectSucceed) break;
        }
      }
    }

    ManipulationPlanner::ManipulationPlanner (const Problem& problem,
        const core::RoadmapPtr_t& roadmap) :
      core::PathPlanner (problem, roadmap),
      shooter_ (new core::BasicConfigurationShooter (problem.robot ())),
      problem_ (problem), pathProjector_ (new pathProjector::Dichotomy (problem_.distance (), 0.1)),
      qProj_ (problem.robot ()->configSize ())
    {}

    void ManipulationPlanner::init (const ManipulationPlannerWkPtr_t& weak)
    {
      core::PathPlanner::init (weak);
      weakPtr_ = weak;
    }
  } // namespace manipulation
} // namespace hpp
