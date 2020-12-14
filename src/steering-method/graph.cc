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

#include "hpp/manipulation/steering-method/graph.hh"

#include <hpp/util/pointer.hh>

#include <hpp/core/straight-path.hh>
#include <hpp/core/steering-method/straight.hh>

#include <hpp/manipulation/problem.hh>
#include <hpp/manipulation/graph/graph.hh>
#include <hpp/manipulation/graph/edge.hh>

namespace hpp {
  namespace manipulation {
    SteeringMethod::SteeringMethod (const ProblemConstPtr_t& problem) :
      core::SteeringMethod (problem), problem_ (problem),
      steeringMethod_ (core::SteeringMethodStraight::create (problem))
    {
    }

    SteeringMethod::SteeringMethod (const SteeringMethod& other):
      core::SteeringMethod (other), problem_ (other.problem_), steeringMethod_
      (other.steeringMethod_)
    {
    }

    namespace steeringMethod {

      GraphPtr_t Graph::create
        (const core::ProblemConstPtr_t& problem)
        {
          assert(HPP_DYNAMIC_PTR_CAST (const Problem, problem));
          ProblemConstPtr_t p = HPP_STATIC_PTR_CAST(const Problem, problem);
          Graph* ptr = new Graph (p);
          GraphPtr_t shPtr (ptr);
          ptr->init (shPtr);
          return shPtr;
        }

      GraphPtr_t Graph::createCopy
        (const GraphPtr_t& other)
        {
          Graph* ptr = new Graph (*other);
          GraphPtr_t shPtr (ptr);
          ptr->init (shPtr);
          return shPtr;
        }

      Graph::Graph (const ProblemConstPtr_t& problem) :
        SteeringMethod (problem), weak_ ()
      {
      }

      Graph::Graph (const Graph& other):
        SteeringMethod (other)
      {
      }

      PathPtr_t Graph::impl_compute (ConfigurationIn_t q1, ConfigurationIn_t q2) const
      {
        graph::Edges_t possibleEdges;
        // If q1 and q2 are the same, call the problem steering method between
        // them
        if (q1 == q2) {
          core::SteeringMethodPtr_t sm
            (problem_->manipulationSteeringMethod()->innerSteeringMethod());
          return (*sm) (q1, q2);
        }
        if (!problem_->constraintGraph())
          throw std::invalid_argument ("The constraint graph should be set to use the steeringMethod::Graph");
        const graph::Graph& graph = *(problem_->constraintGraph ());
        try {
          possibleEdges = graph.getEdges
            (graph.getState (q1), graph.getState (q2));
        } catch (const std::logic_error& e) {
          hppDout (error, e.what ());
          return PathPtr_t ();
        }
        PathPtr_t path;
        if (possibleEdges.empty()) {
          hppDout (info, "No edge found.");
        }
        while (!possibleEdges.empty()) {
          if (possibleEdges.back ()->build (path, q1, q2)) {
            return path;
          }
          possibleEdges.pop_back ();
        }
        return PathPtr_t ();
      }
    } // namespace steeringMethod
  } // namespace manipulation
} // namespace hpp
