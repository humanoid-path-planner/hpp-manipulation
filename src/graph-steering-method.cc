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

#include "hpp/manipulation/graph-steering-method.hh"

#include <hpp/core/straight-path.hh>

#include "hpp/manipulation/graph/graph.hh"
#include "hpp/manipulation/graph/edge.hh"

namespace hpp {
  namespace manipulation {
    GraphSteeringMethodPtr_t GraphSteeringMethod::create
    (const model::DevicePtr_t& robot)
    {
      GraphSteeringMethod* ptr = new GraphSteeringMethod (robot);
      GraphSteeringMethodPtr_t shPtr (ptr);
      ptr->init (shPtr);
      return shPtr;
    }

    GraphSteeringMethodPtr_t GraphSteeringMethod::createCopy
    (const GraphSteeringMethodPtr_t& other)
    {
      GraphSteeringMethod* ptr = new GraphSteeringMethod (*other);
      GraphSteeringMethodPtr_t shPtr (ptr);
      ptr->init (shPtr);
      return shPtr;
    }

    GraphSteeringMethod::GraphSteeringMethod (const model::DevicePtr_t& robot) :
      SteeringMethod (), graph_ (), robot_ (robot),
          distance_ (core::WeighedDistance::create (robot)), weak_ ()
    {
    }

    GraphSteeringMethod::GraphSteeringMethod (const GraphSteeringMethod& other):
      SteeringMethod (other), graph_ (other.graph_), robot_ (other.robot_),
      distance_ (other.distance_)
    {
    }

    PathPtr_t GraphSteeringMethod::impl_compute (ConfigurationIn_t q1, ConfigurationIn_t q2) const
    {
      graph::Edges_t possibleEdges;
      try {
        possibleEdges = graph_->getEdges (graph_->getNode (q1), graph_->getNode (q2));
      } catch (const std::logic_error& e) {
        hppDout (error, e.what ());
        return PathPtr_t ();
      }
      PathPtr_t path;
      while (!possibleEdges.empty()) {
        if (possibleEdges.back ()->build (path, q1, q2, *distance_)) {
          return path;
        }
        possibleEdges.pop_back ();
      }
      return PathPtr_t ();
    }

    const core::WeighedDistancePtr_t& GraphSteeringMethod::distance () const
    {
      return distance_;
    }
  } // namespace manipulation
} // namespace hpp
