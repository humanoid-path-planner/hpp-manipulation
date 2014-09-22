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

#include "hpp/manipulation/graph/graph.hh"
#include "hpp/manipulation/graph/edge.hh"
#include "hpp/manipulation/graph-steering-method.hh"

namespace hpp {
  namespace manipulation {
    GraphSteeringMethod::GraphSteeringMethod (const DevicePtr_t& robot) :
          SteeringMethod (), robot_ (robot),
          distance_ (core::WeighedDistance::create (robot))
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
  } // namespace manipulation
} // namespace hpp
