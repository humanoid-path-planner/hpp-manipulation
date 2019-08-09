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

#include <hpp/manipulation/weighed-distance.hh>

#include <hpp/util/debug.hh>

#include <hpp/manipulation/device.hh>
#include <hpp/manipulation/roadmap-node.hh>
#include <hpp/manipulation/graph/graph.hh>
#include <hpp/manipulation/graph/edge.hh>

namespace hpp {
  namespace manipulation {
    WeighedDistancePtr_t WeighedDistance::create
      (const DevicePtr_t& robot, const graph::GraphPtr_t& graph)
    {
      WeighedDistance* ptr = new WeighedDistance (robot, graph);
      WeighedDistancePtr_t shPtr (ptr);
      ptr->init (shPtr);
      return shPtr;
    }

    WeighedDistancePtr_t WeighedDistance::createCopy
	(const WeighedDistancePtr_t& distance)
    {
      WeighedDistance* ptr = new WeighedDistance (*distance);
      WeighedDistancePtr_t shPtr (ptr);
      ptr->init (shPtr);
      return shPtr;
    }

    core::DistancePtr_t WeighedDistance::clone () const
    {
      return createCopy (weak_.lock ());
    }

    WeighedDistance::WeighedDistance (const DevicePtr_t& robot,
        const graph::GraphPtr_t graph) :
      core::WeighedDistance (robot), graph_ (graph)
    {
    }

    WeighedDistance::WeighedDistance (const WeighedDistance& distance) :
      core::WeighedDistance (distance), graph_ (distance.graph_)
    {
    }

    void WeighedDistance::init (WeighedDistanceWkPtr_t self)
    {
      weak_ = self;
    }

    value_type WeighedDistance::impl_distance (ConfigurationIn_t q1,
					       ConfigurationIn_t q2) const
    {
      value_type d = core::WeighedDistance::impl_distance (q1, q2);
      return d;

      // graph::Edges_t pes = graph_->getEdges
        // (graph_->getNode (q1), graph_->getNode (q2));
      // while (!pes.empty ()) {
        // if (pes.back ()->canConnect (q1, q2))
          // return d;
        // pes.pop_back ();
      // }
      // return d + 100;
    }

    value_type WeighedDistance::impl_distance (core::NodePtr_t n1,
					       core::NodePtr_t n2) const
    {
      Configuration_t& q1 = *n1->configuration(),
                       q2 = *n2->configuration();
      value_type d = core::WeighedDistance::impl_distance (q1, q2);

      graph::Edges_t pes = graph_->getEdges (
          graph_->getState (static_cast <RoadmapNodePtr_t>(n1)),
          graph_->getState (static_cast <RoadmapNodePtr_t>(n2)));
      while (!pes.empty ()) {
        if (pes.back ()->canConnect (q1, q2))
          return d;
        pes.pop_back ();
      }
      return d + 100;
    }
  } //   namespace manipulation
} // namespace hpp
