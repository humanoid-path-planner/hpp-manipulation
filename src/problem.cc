// Copyright (c) 2015, Joseph Mirabel
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

#include <hpp/manipulation/problem.hh>

#include <hpp/core/path-validation/discretized-collision-checking.hh>

#include <hpp/manipulation/weighed-distance.hh>
#include <hpp/manipulation/steering-method/graph.hh>
#include <hpp/manipulation/graph-path-validation.hh>
#include <hpp/manipulation/graph/graph.hh>

namespace hpp {
  namespace manipulation {
    ProblemPtr_t Problem::create (DevicePtr_t robot)
    {
      ProblemPtr_t p (new Problem (robot));
      p->init(p);
      return p;
    }

    Problem::Problem (DevicePtr_t robot)
      : Parent (robot), graph_()
    {
    }

    void Problem::init (ProblemWkPtr_t wkPtr)
    {
      Parent::init (wkPtr);
      wkPtr_ = wkPtr;

      Parent::steeringMethod (steeringMethod::Graph::create (wkPtr_.lock()));
      distance (WeighedDistance::create (HPP_DYNAMIC_PTR_CAST(Device, robot()), graph_));
      setPathValidationFactory(core::pathValidation::createDiscretizedCollisionChecking, 0.05);
    }

    void Problem::constraintGraph (const graph::GraphPtr_t& graph)
    {
      graph_ = graph;
      graph_->problem (wkPtr_.lock());

      GraphPathValidationPtr_t pv = HPP_DYNAMIC_PTR_CAST(GraphPathValidation, pathValidation());
      if (pv) pv->constraintGraph (graph_);
      WeighedDistancePtr_t d = HPP_DYNAMIC_PTR_CAST (WeighedDistance,
          distance ());
      if (d) d->constraintGraph (graph);
    }

    PathValidationPtr_t Problem::pathValidation () const
    {
      return Parent::pathValidation();
    }

    void Problem::pathValidation (const PathValidationPtr_t& pathValidation)
    {
      GraphPathValidationPtr_t pv = HPP_DYNAMIC_PTR_CAST(GraphPathValidation, pathValidation);
      if (pv) pv->constraintGraph (graph_);
      Parent::pathValidation (pathValidation);
    }

    PathValidationPtr_t Problem::pathValidationFactory () const
    {
      PathValidationPtr_t pv (pvFactory_ (robot(), pvTol_));

      shared_ptr<core::ObstacleUserInterface> oui =
        HPP_DYNAMIC_PTR_CAST(core::ObstacleUserInterface, pv);
      if (oui) {
        const core::ObjectStdVector_t& obstacles (collisionObstacles ());
        // Insert obstacles in path validation object
        for (core::ObjectStdVector_t::const_iterator _obs = obstacles.begin ();
            _obs != obstacles.end (); ++_obs)
          oui->addObstacle (*_obs);
      }
      GraphPathValidationPtr_t gpv = HPP_DYNAMIC_PTR_CAST(GraphPathValidation, pv);
      if (gpv) return gpv->innerValidation();
      return pv;
    }

    SteeringMethodPtr_t Problem::manipulationSteeringMethod () const
    {
      return HPP_DYNAMIC_PTR_CAST (SteeringMethod,
          Parent::steeringMethod());
    }

    void Problem::setPathValidationFactory (
        const core::PathValidationBuilder_t& factory,
        const value_type& tol)
    {
      pvFactory_ = factory;
      pvTol_ = tol;
      if (graph_) graph_->invalidate();
    }

    void Problem::checkProblem () const
    {
      core::Problem::checkProblem ();
      if (!graph_)
        throw std::runtime_error ("No graph in the problem.");
    }
  } // namespace manipulation
} // namespace hpp
