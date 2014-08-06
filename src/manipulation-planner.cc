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

#include "hpp/manipulation/robot.hh"
#include "hpp/manipulation/problem.hh"
#include "hpp/manipulation/manipulation-planner.hh"

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
          itcc != roadmap ()->connectedComponents ().end (); itcc++) {
        // Find the nearest neighbor.
        core::value_type distance;
        core::NodePtr_t near = roadmap ()->nearestNode (q_rand, *itcc, distance);

        bool pathIsValid = extend (near->configuration (), q_rand, path);
        // Insert new path to q_near in roadmap
        value_type t_final = path->timeRange ().second;
        if (t_final != path->timeRange ().first) {
          ConfigurationPtr_t q_new (new Configuration_t
              ((*path) (t_final)));
          if (!pathIsValid || !belongs (q_new, newNodes)) {
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

      // Try to connect the new nodes together
      const core::SteeringMethodPtr_t& sm (problem ().steeringMethod ());
      core::PathValidationPtr_t pathValidation (problem ().pathValidation ());
      core::PathPtr_t validPath;
      for (core::Nodes_t::const_iterator itn1 = newNodes.begin ();
          itn1 != newNodes.end (); ++itn1) {
        for (core::Nodes_t::const_iterator itn2 = boost::next (itn1);
            itn2 != newNodes.end (); ++itn2) {
          ConfigurationPtr_t q1 ((*itn1)->configuration ());
          ConfigurationPtr_t q2 ((*itn2)->configuration ());
          assert (*q1 != *q2);
          path = (*sm) (*q1, *q2);
          if (pathValidation->validate (path, false, validPath)) {
            roadmap ()->addEdge (*itn1, *itn2, path);
            core::interval_t timeRange = path->timeRange ();
            roadmap ()->addEdge (*itn2, *itn1, path->extract
                (core::interval_t (timeRange.second,
                                   timeRange.first)));
          }
        }
      }
    }

    bool ManipulationPlanner::extend(
        const ConfigurationPtr_t& q_near,
        const ConfigurationPtr_t& q_rand,
        core::PathPtr_t& validPath)
    {
      graph::GraphPtr_t graph = problem_.constraintGraph ();
      // Select next node in the constraint graph.
      graph::Nodes_t nodes = graph->getNode (*q_near);
      graph::Edges_t edges = graph->chooseEdge (nodes);
      ConstraintPtr_t constraint = graph->configConstraint (edges, *q_near);
      qProj_ = *q_rand;
      if (!constraint->apply (qProj_))
        return false;
      core::SteeringMethodPtr_t sm (problem().steeringMethod());
      core::PathPtr_t path = (*sm) (*q_near, qProj_);
      if (!path)
        return false;
      core::PathValidationPtr_t pathValidation (problem ().pathValidation ());
      pathValidation->validate (path, false, validPath);
      return true;
    }

    ManipulationPlanner::ManipulationPlanner (const Problem& problem,
        const core::RoadmapPtr_t& roadmap) :
      core::PathPlanner (problem, roadmap),
      shooter_ (new core::BasicConfigurationShooter (problem.robot ())),
      problem_ (problem),
      qProj_ (problem.robot ()->configSize ())
    {}

    void ManipulationPlanner::init (const ManipulationPlannerWkPtr_t& weak)
    {
      core::PathPlanner::init (weak);
      weakPtr_ = weak;
    }
  } // namespace manipulation
} // namespace hpp
