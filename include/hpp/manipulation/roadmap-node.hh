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
          cacheSystem_ (defaultCachingSystem),
          state_ ()
        {}

        RoadmapNode (const ConfigurationPtr_t& configuration,
            ConnectedComponentPtr_t cc);

        /// \name Cache system
        /// \{

        enum CachingSystem {
          /// The caching system is disabled. The graph::Node containing this
          /// node can be obtained by checking the constraints.
          CACHE_DISABLED,
          /// The chaching system is enabled and up to date.
          CACHE_UP_TO_DATE,
          /// The chaching system is enabled but the cache is not up to date.
          CACHE_NEED_UPDATE
        };

        static CachingSystem defaultCachingSystem;

        /// Get the caching system being used.
        CachingSystem cachingSystem () const
        {
          return cacheSystem_;
        }

        /// Getter for the graph::State.
	/// \deprecated use graphState instead.
        graph::StatePtr_t graphNode () const HPP_MANIPULATION_DEPRECATED
        {
          return state_.lock();
        }

        /// Getter for the graph::State.
        graph::StatePtr_t graphState () const
        {
          return state_.lock();
        }

        /// Setter for the graph::State.
	/// \deprecated use graphState instead
        void graphNode (const graph::StatePtr_t& state)
	  HPP_MANIPULATION_DEPRECATED
        {
          if (cacheSystem_ != CACHE_DISABLED) cacheSystem_ = CACHE_UP_TO_DATE;
          state_ = state;
        }

        /// Setter for the graph::State.
        void graphState (const graph::StatePtr_t& state)
        {
          if (cacheSystem_ != CACHE_DISABLED) cacheSystem_ = CACHE_UP_TO_DATE;
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
        CachingSystem cacheSystem_;

        graph::StateWkPtr_t state_;
        LeafConnectedCompPtr_t leafCC_;
    };
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_ROADMAP_NODE_HH
