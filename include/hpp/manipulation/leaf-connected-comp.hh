//
// Copyright (c) 2016 CNRS
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.

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
        typedef LeafConnectedComp* RawPtr_t;
        typedef std::set <RawPtr_t> LeafConnectedComps_t;
        /// return a shared pointer to new instance
        static LeafConnectedCompPtr_t create (const RoadmapPtr_t& roadmap);

        /// Merge two connected components
        /// \param other manipulation symbolic component to merge into this one.
        /// \note other will be empty after calling this method.
        virtual void merge (const LeafConnectedCompPtr_t& otherCC);

        /// Whether this connected component can reach cc
        /// \param cc a connected component
        bool canReach (const LeafConnectedCompPtr_t& cc);

        /// Whether this connected component can reach cc
        /// \param cc a connected component
        /// \retval cc2Tocc1 list of connected components between cc2 and cc1
        ///         that should be merged.
        bool canReach (const LeafConnectedCompPtr_t& cc,
                       LeafConnectedComp::LeafConnectedComps_t& cc2Tocc1);

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

        LeafConnectedCompPtr_t self ()
        {
          return weak_.lock ();
        }

        const LeafConnectedComp::LeafConnectedComps_t& from () const
        {
          return from_;
        }

        const LeafConnectedComp::LeafConnectedComps_t& to () const
        {
          return to_;
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

        /// For serialization only.
        LeafConnectedComp() {}

      private:
        static void clean (LeafConnectedComps_t& set);
        // status variable to indicate whether or not CC has been visited
        mutable bool explored_;
        RoadmapWkPtr_t roadmap_;
        LeafConnectedComps_t to_, from_;
        LeafConnectedCompWkPtr_t weak_;
        friend class Roadmap;

        HPP_SERIALIZABLE();
    }; // class LeafConnectedComp

    class HPP_MANIPULATION_DLLAPI WeighedLeafConnectedComp :
      public LeafConnectedComp
    {
      public:
        void merge (const LeafConnectedCompPtr_t& otherCC);

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
        WeighedLeafConnectedComp() {}
        HPP_SERIALIZABLE();
    }; // class LeafConnectedComp
  } //   namespace manipulation
} // namespace hpp
#endif // HPP_MANIPULATION_LEAF_CONNECTED_COMP_HH
