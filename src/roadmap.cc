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

#include <hpp/util/pointer.hh>

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
      Histograms newHistograms;
      Histograms::iterator it;
      for (it = histograms_.begin(); it != histograms_.end(); it++) {
        newHistograms.push_back ((*it)->clone ());
      }
      histograms_ = newHistograms;
    }

    void Roadmap::push_node (const core::NodePtr_t& n)
    {
      Parent::push_node (n);
      statInsert (n);
    }

    void Roadmap::statInsert (const core::NodePtr_t& n)
    {
      Histograms::iterator it;
      for (it = histograms_.begin(); it != histograms_.end(); it++) {
        (*it)->add (n);
      }
    }

    void Roadmap::statAddFoliation (ConstraintSetPtr_t constraint)
    {
      insertHistogram (graph::HistogramPtr_t (new graph::LeafHistogram (constraint)));
    }

    void Roadmap::insertHistogram (const graph::HistogramPtr_t hist)
    {
      histograms_.push_back (hist);
    }

    void Roadmap::constraintGraph (const graph::GraphPtr_t& graph)
    {
      Histograms::iterator it = histograms_.begin();
      for (; it != histograms_.end();) {
        if (HPP_DYNAMIC_PTR_CAST (graph::NodeHistogram, *it))
          it = histograms_.erase (it);
        else
          it++;
      }
      insertHistogram (graph::HistogramPtr_t (new graph::NodeHistogram (graph)));
    }
  } // namespace manipulation
} // namespace hpp
