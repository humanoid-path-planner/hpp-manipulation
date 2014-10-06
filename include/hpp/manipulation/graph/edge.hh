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

#ifndef HPP_MANIPULATION_GRAPH_EDGE_HH
# define HPP_MANIPULATION_GRAPH_EDGE_HH

#include <hpp/core/constraint-set.hh>
#include <hpp/core/weighed-distance.hh>
#include <hpp/core/path.hh>

#include "hpp/manipulation/config.hh"
#include "hpp/manipulation/fwd.hh"
#include "hpp/manipulation/graph/graph.hh"

namespace hpp {
  namespace manipulation {
    namespace graph {
      /// Cache mechanism that enable const-correctness of member functions.
      template <typename C>
        class HPP_MANIPULATION_LOCAL Cache
      {
        public:
          void set (const C& c)
          {
            c_ = c;
          }

          operator bool() const
          {
            return (bool)c_;
          }

          const C& get () const
          {
            return c_;
          }

        private:
          C c_;
      };

      /// Abstract class representing representing the link between two nodes.
      class HPP_MANIPULATION_DLLAPI Edge : public GraphComponent
      {
        public:
          /// Destructor
          ~Edge ();

          /// Create a new empty Edge.
          static EdgePtr_t create (const GraphWkPtr_t& graph, const NodeWkPtr_t& from, const NodeWkPtr_t& to);

          virtual bool applyConstraints (core::NodePtr_t nnear, ConfigurationOut_t q) const;

          virtual bool applyConstraints (ConfigurationIn_t qoffset, ConfigurationOut_t q) const;

          virtual bool build (core::PathPtr_t& path, ConfigurationIn_t q1, ConfigurationIn_t q2, const core::WeighedDistance& d) const;

          /// Get the destination
          NodePtr_t to () const;

          /// Get the origin
          NodePtr_t from () const;

          /// Get the node in which path is.
          NodePtr_t node () const;

          void isInNodeFrom (bool iinf)
          {
            isInNodeFrom_ = iinf;
          }

          bool isInNodeFrom () const
          {
            return isInNodeFrom_;
          }
        protected:
          /// Initialization of the object.
          void init (const EdgeWkPtr_t& weak, const GraphWkPtr_t& graph, const NodeWkPtr_t& from,
              const NodeWkPtr_t& to);

          /// Constructor
          Edge();

        private:
          typedef Cache < ConstraintSetPtr_t > Constraint_t;

          /// See pathConstraint member function.
          Constraint_t* pathConstraints_;

          /// Constraint ensuring that a q_proj will be in to_ and in the
          /// same leaf of to_ as the configuration used for initialization.
          Constraint_t* configConstraints_;

          /// The two ends of the transition.
          NodeWkPtr_t from_, to_;

          /// True if this path is in node from, False if in node to
          bool isInNodeFrom_;

          /// Weak pointer to itself.
          EdgeWkPtr_t wkPtr_;

          /// Constraint to project onto the same leaf as config.
          /// \return The initialized projector.
          ConstraintSetPtr_t configConstraint() const;

          /// Constraint to project a path.
          /// \return The initialized constraint.
          ConstraintSetPtr_t pathConstraint() const;

          /// Print the object in a stream.
          virtual std::ostream& print (std::ostream& os) const;

          friend class Graph;
      }; // class Edge

      /// Edge with waypoint
      class HPP_MANIPULATION_DLLAPI WaypointEdge : public Edge
      {
        public:
          /// Create a new WaypointEdge.
          static WaypointEdgePtr_t create (const GraphWkPtr_t& graph, const NodeWkPtr_t& from, const NodeWkPtr_t& to);

          virtual bool build (core::PathPtr_t& path, ConfigurationIn_t q1, ConfigurationIn_t q2, const core::WeighedDistance& d) const;

          virtual bool applyConstraints (ConfigurationIn_t qoffset, ConfigurationOut_t q) const;

          EdgePtr_t waypoint () const;

        protected:
          /// Initialization of the object.
          void init (const EdgeWkPtr_t& weak, const GraphWkPtr_t& graph, const NodeWkPtr_t& from,
              const NodeWkPtr_t& to);

        private:
          typedef std::pair < EdgePtr_t, NodePtr_t > Waypoint;

          void createWaypoint ();

          Waypoint waypoint_;
          mutable Configuration_t config_;

          /// Print the object in a stream.
          virtual std::ostream& print (std::ostream& os) const;
      }; // class WaypointEdge

      /// Edge that find intersection of level set.
      class HPP_MANIPULATION_DLLAPI LevelSetEdge : public Edge
      {
        public:
          /// Create a new LevelSetEdge.
          static LevelSetEdgePtr_t create (const GraphWkPtr_t& graph, const NodeWkPtr_t& from, const NodeWkPtr_t& to);

          virtual bool applyConstraints (ConfigurationIn_t qoffset, ConfigurationOut_t q) const;

          virtual bool applyConstraints (core::NodePtr_t n_offset, ConfigurationOut_t q) const;

          void histogram (LeafHistogramPtr_t hist);

        protected:
          /// Initialization of the object.
          void init (const EdgeWkPtr_t& weak, const GraphWkPtr_t& graph, const NodeWkPtr_t& from,
              const NodeWkPtr_t& to);

        private:
          /// Print the object in a stream.
          virtual std::ostream& print (std::ostream& os) const;

          /// This histogram will be used to find a good level set.
          LeafHistogramPtr_t hist_;
      }; // class LevelSetEdge
    } // namespace graph
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_GRAPH_EDGE_HH
