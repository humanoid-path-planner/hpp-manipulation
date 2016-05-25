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

#ifndef HPP_MANIPULATION_SYMBOLIC_COMPONENT_HH
#define HPP_MANIPULATION_SYMBOLIC_COMPONENT_HH

#include <hpp/manipulation/config.hh>
#include <hpp/manipulation/fwd.hh>
#include <hpp/manipulation/graph/fwd.hh>

#include <hpp/manipulation/roadmap-node.hh>

namespace hpp {
  namespace manipulation {
    /// Set of configurations accessible to each others by a single edge,
    /// with the same right hand side.
    ///
    /// This assumes the roadmap is not directed.
    class HPP_MANIPULATION_DLLAPI SymbolicComponent
    {
      public:
        typedef std::set<SymbolicComponentPtr_t> SymbolicComponents_t;

        /// return a shared pointer to new instance
        static SymbolicComponentPtr_t create (const RoadmapPtr_t& roadmap);

        /// Merge two symbolic components
        /// \param other manipulation symbolic component to merge into this one.
        /// \note other will be empty after calling this method.
        virtual void merge (SymbolicComponentPtr_t otherCC);

        bool canMerge (const SymbolicComponentPtr_t& otherCC) const;

        /// Add otherCC to the list of reachable components
        ///
        /// This also add this object to the list of ancestor of otherCC.
        void canReach (const SymbolicComponentPtr_t& otherCC);

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
        SymbolicComponent(const RoadmapPtr_t& r)
          : roadmap_(r) {}

        void init (const SymbolicComponentWkPtr_t& shPtr)
        {
          weak_ = shPtr;
        }

        graph::NodePtr_t state_;
        RoadmapNodes_t nodes_;

      private:
        RoadmapPtr_t roadmap_;
        SymbolicComponents_t to_, from_;
        SymbolicComponentWkPtr_t weak_;
    }; // class SymbolicComponent

    class HPP_MANIPULATION_DLLAPI WeighedSymbolicComponent :
      public SymbolicComponent
    {
      public:
        void merge (SymbolicComponentPtr_t otherCC);

        void setFirstNode (const RoadmapNodePtr_t& node);

        static WeighedSymbolicComponentPtr_t create (const RoadmapPtr_t& roadmap);

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
        WeighedSymbolicComponent(const RoadmapPtr_t& r)
          : SymbolicComponent(r), weight_(1) {}

      private:
    }; // class SymbolicComponent
  } //   namespace manipulation
} // namespace hpp
#endif // HPP_MANIPULATION_SYMBOLIC_COMPONENT_HH
