// Copyright (c) 2014 CNRS
// Authors: Joseph Mirabel
//
//
// This file is part of hpp-manipulation.
// hpp-manipulation is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-manipulation is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-manipulation. If not, see
// <http://www.gnu.org/licenses/>.

#ifndef HPP_MANIPULATION_ROADMAP_NODE_HH
# define HPP_MANIPULATION_ROADMAP_NODE_HH

# include <hpp/core/node.hh>

# include <hpp/manipulation/fwd.hh>
# include <hpp/manipulation/deprecated.hh>
# include <hpp/manipulation/config.hh>
# include <hpp/manipulation/graph/fwd.hh>
# include <hpp/manipulation/connected-component.hh>

namespace hpp {
  namespace manipulation {
    class HPP_MANIPULATION_DLLAPI RoadmapNode : public core::Node
    {
      public:
        RoadmapNode (const ConfigurationPtr_t& configuration) :
          core::Node (configuration),
          state_ ()
        {}

        RoadmapNode (const ConfigurationPtr_t& configuration,
            ConnectedComponentPtr_t cc);

        /// \name Cache
        /// \{

        /// Get the caching system being used.
        bool cacheUpToDate () const
        {
          return static_cast<bool>(graphState());
        }

        /// Getter for the graph::State.
        graph::StatePtr_t graphState () const
        {
          return state_.lock();
        }

        /// Setter for the graph::State.
        void graphState (const graph::StatePtr_t& state)
        {
          state_ = state;
        }
        /// \}

        void leafConnectedComponent (const LeafConnectedCompPtr_t& sc)
        {
          leafCC_ = sc;
        }

        LeafConnectedCompPtr_t leafConnectedComponent () const
        {
          return leafCC_;
        }

      private:
        graph::StateWkPtr_t state_;
        LeafConnectedCompPtr_t leafCC_;

        RoadmapNode() {}
        HPP_SERIALIZABLE();
    };
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_ROADMAP_NODE_HH
