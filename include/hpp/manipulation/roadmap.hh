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
# include "hpp/manipulation/graph/fwd.hh"
# include "hpp/manipulation/graph/statistics.hh"

namespace hpp {
  namespace manipulation {
    /// \addtogroup roadmap
    /// \{

    /// Extension of hpp::core::Roadmap. It adds the ability of doing
    /// statistics on the graph
    class HPP_MANIPULATION_DLLAPI Roadmap : public core::Roadmap
    {
      public:
        typedef core::Roadmap Parent;

        /// Return a shared pointer to a new instance
        static RoadmapPtr_t create (const core::DistancePtr_t& distance, const core::DevicePtr_t& robot);

        /// Register histogram so that each time a node is added to the roadmap,
        /// it is also added to the histogram
        void insertHistogram (const graph::HistogramPtr_t hist);

        /// Register the constraint graph to do statistics.
        void constraintGraph (const graph::GraphPtr_t& graph);

        /// Clear the histograms and call parent implementation.
        void clear ();

        /// Catch event 'New node added'
        void push_node (const core::NodePtr_t& n);

        /// Get the nearest neighbor in a given graph::Node and in a given
        /// ConnectedComponent.
        RoadmapNodePtr_t nearestNode (const ConfigurationPtr_t& configuration,
            const ConnectedComponentPtr_t& connectedComponent,
            const graph::NodePtr_t& node,
            value_type& minDistance) const;

	/// Get graph node corresponding to given roadmap node
 	graph::NodePtr_t getNode(RoadmapNodePtr_t node);

      protected:
        /// Register a new configuration.
        void statInsert (const RoadmapNodePtr_t& n);

        /// Constructor
        Roadmap (const core::DistancePtr_t& distance, const core::DevicePtr_t& robot);

        /// Node factory
        core::NodePtr_t createNode (const ConfigurationPtr_t& config) const;

        void init (const RoadmapPtr_t& shPtr)
	{
	  weak_ = shPtr;
	}
        
      private:
        typedef std::list < graph::HistogramPtr_t > Histograms;
        /// Keep track of the leaf that are explored.
        /// There should be one histogram per foliation.
        Histograms histograms_;
        graph::GraphPtr_t graph_;
        RoadmapWkPtr_t weak_;
    };
    /// \}
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_ROADMAP_HH
