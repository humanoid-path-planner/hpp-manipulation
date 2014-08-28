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

#include "hpp/manipulation/roadmap.hh"

namespace hpp {
  namespace manipulation {
    Roadmap::Roadmap (const core::DistancePtr_t& distance, const core::DevicePtr_t& robot) :
      core::Roadmap (distance, robot) {}

    RoadmapPtr_t Roadmap::create (const core::DistancePtr_t& distance, const core::DevicePtr_t& robot)
    {
      return RoadmapPtr_t (new Roadmap (distance, robot));
    }

    void Roadmap::clear ()
    {
      Parent::clear ();
      std::vector < graph::HistogramPtr_t > newHistograms;
      std::vector < graph::HistogramPtr_t >::iterator it;
      for (it = histograms_.begin(); it != histograms_.end(); it++) {
        newHistograms.push_back ((*it)->clone ());
      }
      histograms_ = newHistograms;
    }

    void Roadmap::push_node (const core::NodePtr_t& n)
    {
      Parent::push_node (n);
      statInsert (*(n->configuration ()));
    }

    void Roadmap::statInsert (ConfigurationIn_t config)
    {
      std::vector < graph::HistogramPtr_t >::iterator it;
      for (it = histograms_.begin(); it != histograms_.end(); it++) {
        (*it)->add (config);
      }
    }

    void Roadmap::statAddFoliation (ConstraintSetPtr_t constraint)
    {
      histograms_.push_back (graph::HistogramPtr_t (new graph::LeafHistogram (constraint)));
    }

    void Roadmap::constraintGraph (const graph::GraphPtr_t& graph)
    {
      histograms_.push_back (graph::HistogramPtr_t (new graph::NodeHistogram (graph)));
    }
  } // namespace manipulation
} // namespace hpp
