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

#ifndef HPP_MANIPULATION_ROADMAP_HH
# define HPP_MANIPULATION_ROADMAP_HH

# include <hpp/core/roadmap.hh>
# include <hpp/core/constraint-set.hh>

# include "hpp/manipulation/fwd.hh"
# include "hpp/manipulation/graph/statistics.hh"

namespace hpp {
  namespace manipulation {
    /// Extension of hpp::core::Roadmap. It adds the ability of doing
    /// statistics on the graph
    class HPP_MANIPULATION_DLLAPI Roadmap : public core::Roadmap
    {
      public:
        typedef core::Roadmap Parent;

        /// Return a shared pointer to a new instance
        static RoadmapPtr_t create (const core::DistancePtr_t& distance, const core::DevicePtr_t& robot);

        /// Add a ConstraintSet that creates a foliation.
        void statAddFoliation (ConstraintSetPtr_t constraint);

        /// Register the constraint graph to do statistics.
        void constraintGraph (const graph::GraphPtr_t& graph);

        /// Clear the histograms and call parent implementation.
        void clear ();

        /// Catch event 'New node added'
        void push_node (const core::NodePtr_t& n);

      protected:
        /// Register a new configuration.
        void statInsert (const core::NodePtr_t& n);

        /// Constructor
        Roadmap (const core::DistancePtr_t& distance, const core::DevicePtr_t& robot);

      private:
        typedef std::list < graph::HistogramPtr_t > Histograms;
        /// Keep track of the leaf that are explored.
        /// There should be one histogram per foliation.
        Histograms histograms_;
    };
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_ROADMAP_HH
