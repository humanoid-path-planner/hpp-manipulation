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
    Problem::Problem (DevicePtr_t robot)
      : Parent (robot), graph_()
    {
      Parent::steeringMethod (steeringMethod::Graph::create (*this));
      distance (WeighedDistance::create (robot, graph_));
      setPathValidationFactory(core::pathValidation::createDiscretizedCollisionChecking, 0.05);

      // add<boost::any>("ManipulationPlanner/ExtendStep", (value_type)1);
    }

    void Problem::constraintGraph (const graph::GraphPtr_t& graph)
    {
      graph_ = graph;
      graph_->problem (this);
      if (pathValidation ())
        pathValidation ()->constraintGraph (graph);
      WeighedDistancePtr_t d = HPP_DYNAMIC_PTR_CAST (WeighedDistance,
          distance ());
      if (d) d->constraintGraph (graph);
    }

    GraphPathValidationPtr_t Problem::pathValidation () const
    {
      return HPP_DYNAMIC_PTR_CAST (GraphPathValidation,
          Parent::pathValidation());
    }

    void Problem::pathValidation (const PathValidationPtr_t& pathValidation)
    {
      GraphPathValidationPtr_t pv (GraphPathValidation::create (pathValidation));
      pv->constraintGraph (graph_);
      Parent::pathValidation (pv);
    }

    PathValidationPtr_t Problem::pathValidationFactory () const
    {
      PathValidationPtr_t pv (pvFactory_ (robot(), pvTol_));
      const core::ObjectStdVector_t& obstacles (collisionObstacles ());
      // Insert obstacles in path validation object
      for (core::ObjectStdVector_t::const_iterator _obs = obstacles.begin ();
	   _obs != obstacles.end (); ++_obs)
	pv->addObstacle (*_obs);
      return pv;
    }

    SteeringMethodPtr_t Problem::steeringMethod () const
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
  } // namespace manipulation
} // namespace hpp
