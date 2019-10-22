// Copyright (c) 2015, Joseph Mirabel
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

      Parent::steeringMethod (steeringMethod::Graph::create (*this));
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
      const core::ObjectStdVector_t& obstacles (collisionObstacles ());
      // Insert obstacles in path validation object
      for (core::ObjectStdVector_t::const_iterator _obs = obstacles.begin ();
	   _obs != obstacles.end (); ++_obs)
	pv->addObstacle (*_obs);
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
      if (graph_) graph_->setDirty();
    }

    void Problem::checkProblem () const
    {
      core::Problem::checkProblem ();
      if (!graph_)
        throw std::runtime_error ("No graph in the problem.");
    }
  } // namespace manipulation
} // namespace hpp
