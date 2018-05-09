//
// Copyright (c) 2016 CNRS
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//
// This file is part of hpp-manipulation
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
// hpp-manipulation  If not, see
// <http://www.gnu.org/licenses/>.

#ifndef HPP_MANIPULATION_LEAF_CONNECTED_COMP_HH
#define HPP_MANIPULATION_LEAF_CONNECTED_COMP_HH

#include <hpp/manipulation/config.hh>
#include <hpp/manipulation/fwd.hh>
#include <hpp/manipulation/graph/fwd.hh>

#include <hpp/manipulation/roadmap-node.hh>

namespace hpp {
  namespace manipulation {
    /// Set of configurations accessible to each others by a single transition,
    /// with the same right hand side.
    ///
    /// This assumes the roadmap is not directed.
    class HPP_MANIPULATION_DLLAPI LeafConnectedComp
    {
      public:
        typedef std::set<LeafConnectedComp*> LeafConnectedComps_t;

        /// return a shared pointer to new instance
        static LeafConnectedCompPtr_t create (const RoadmapPtr_t& roadmap);

        /// Merge two symbolic components
        /// \param other manipulation symbolic component to merge into this one.
        /// \note other will be empty after calling this method.
        virtual void merge (LeafConnectedCompPtr_t otherCC);

        bool canMerge (const LeafConnectedCompPtr_t& otherCC) const;

        /// Add otherCC to the list of reachable components
        ///
        /// This also add this object to the list of ancestor of otherCC.
        void canReach (const LeafConnectedCompPtr_t& otherCC);

        /// Add roadmap node to connected component
        /// \param roadmap node to be added
        void addNode (const RoadmapNodePtr_t& node);

        virtual void setFirstNode (const RoadmapNodePtr_t& node);

        core::ConnectedComponentPtr_t connectedComponent () const
        {
          assert (!nodes_.empty());
          return nodes_.front()->connectedComponent();
        };

        const RoadmapNodes_t& nodes() const
        {
          return nodes_;
        }

      protected:
        LeafConnectedComp(const RoadmapPtr_t& r)
          : roadmap_(r) {}

        void init (const LeafConnectedCompWkPtr_t& shPtr)
        {
          weak_ = shPtr;
        }

        graph::StatePtr_t state_;
        RoadmapNodes_t nodes_;

      private:
        RoadmapWkPtr_t roadmap_;
        LeafConnectedComps_t to_, from_;
        LeafConnectedCompWkPtr_t weak_;
    }; // class LeafConnectedComp

    class HPP_MANIPULATION_DLLAPI WeighedLeafConnectedComp :
      public LeafConnectedComp
    {
      public:
        void merge (LeafConnectedCompPtr_t otherCC);

        void setFirstNode (const RoadmapNodePtr_t& node);

        static WeighedLeafConnectedCompPtr_t create (const RoadmapPtr_t& roadmap);

        std::size_t indexOf (const graph::EdgePtr_t e) const;

        void normalizeProba ()
        {
          const value_type s = p_.sum();
          p_ /= s;
        }

        value_type weight_;
        /// Transition probabilities
        vector_t p_;
        std::vector<graph::EdgePtr_t> edges_;

      protected:
        WeighedLeafConnectedComp(const RoadmapPtr_t& r)
          : LeafConnectedComp(r), weight_(1) {}

      private:
    }; // class LeafConnectedComp
  } //   namespace manipulation
} // namespace hpp
#endif // HPP_MANIPULATION_LEAF_CONNECTED_COMP_HH
