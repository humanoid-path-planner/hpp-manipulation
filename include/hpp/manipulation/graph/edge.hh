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

      /// \addtogroup constraint_graph
      /// \{

      /// Abstract class representing representing the link between two nodes.
      class HPP_MANIPULATION_DLLAPI Edge : public GraphComponent
      {
        public:
          /// Destructor
          ~Edge ();

          /// Create a new empty Edge.
          static EdgePtr_t create
	    (const std::string& name,
	     const core::SteeringMethodPtr_t& steeringMethod,
	     const GraphWkPtr_t& graph,
	     const NodeWkPtr_t& from,
	     const NodeWkPtr_t& to);

          virtual bool applyConstraints (core::NodePtr_t nnear, ConfigurationOut_t q) const;

          virtual bool applyConstraints (ConfigurationIn_t qoffset, ConfigurationOut_t q) const;

          virtual bool build (core::PathPtr_t& path, ConfigurationIn_t q1, ConfigurationIn_t q2, const core::WeighedDistance& d) const;

          /// Get the destination
          NodePtr_t to () const;

          /// Get the origin
          NodePtr_t from () const;

          /// Get the node in which path is.
          virtual NodePtr_t node () const;

          void isInNodeFrom (bool iinf)
          {
            isInNodeFrom_ = iinf;
          }

          bool isInNodeFrom () const
          {
            return isInNodeFrom_;
          }
	  /// Get steering method associated to the edge.
	  const core::SteeringMethodPtr_t& steeringMethod () const
	  {
	    return steeringMethod_;
	  }

          /// Print the object in a stream.
          virtual std::ostream& dotPrint (std::ostream& os, dot::DrawingAttributes da = dot::DrawingAttributes ()) const;

          /// Constraint to project onto the same leaf as config.
          /// \return The initialized projector.
          ConstraintSetPtr_t configConstraint() const;

        protected:
          /// Initialization of the object.
          void init (const EdgeWkPtr_t& weak, const GraphWkPtr_t& graph, const NodeWkPtr_t& from,
              const NodeWkPtr_t& to);

          /// Constructor
          Edge (const std::string& name,
		const core::SteeringMethodPtr_t& steeringMethod);

          /// Constraint to project a path.
          /// \return The initialized constraint.
          ConstraintSetPtr_t pathConstraint() const;

          virtual ConstraintSetPtr_t buildConfigConstraint() const;

          virtual ConstraintSetPtr_t buildPathConstraint() const;

          /// Print the object in a stream.
          virtual std::ostream& print (std::ostream& os) const;

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

	  /// Steering method used to create paths associated to the edge
	  core::SteeringMethodPtr_t steeringMethod_;

          /// Weak pointer to itself.
          EdgeWkPtr_t wkPtr_;

          friend class Graph;
      }; // class Edge

      /// Edge with waypoint.
      /// Waypoints are handled recursively, i.e.\ class WaypointEdge contains only a
      /// Node and an Edge, the second Edge being itself.
      /// In this package, the Node in a WaypointEdge is semantically different from other Node
      /// because it does not correspond to a state with different manipulation rules. It has
      /// the same rules as another Node (either Edge::from() or Edge::to()).
      ///
      /// Semantically, a waypoint Node is fully part of the WaypointEdge. When a corresponding path
      /// reaches it, no planning is required to know what to do next. To the contrary, when a path reaches
      /// Edge::from() or Edge::to(), there may be several possibilities.
      ///
      /// \note
      ///   Implementation details: let's say, between the two nodes \f$N_f\f$ and \f$N_t\f$,
      ///   two waypoints are required:
      ///   \f$ N_f \xrightarrow{e_0} n_0 \xrightarrow{e_1} n_1 \xrightarrow{E} N_t\f$.
      ///   The outmost WaypointEdg contains:
      ///   \li from: \f$N_f\f$,
      ///   \li to: \f$N_t\f$,
      ///   \li constraints: those of edge \f$E\f$,
      ///   \li waypoint: \f$(E_1, n_1)\f$.
      ///
      ///   where \f$E_1\f$ is an instance of class WaypointEdge containing:
      ///   \li from: \f$N_f\f$,
      ///   \li to: \f$n_1\f$,
      ///   \li constraints: those of edge \f$e_1\f$,
      ///   \li waypoint: \f$(e_0, n_0)\f$.
      class HPP_MANIPULATION_DLLAPI WaypointEdge : public Edge
      {
        public:
          /// Create a new WaypointEdge.
	static WaypointEdgePtr_t create
	  (const std::string& name,
	   const core::SteeringMethodPtr_t& steeringMethod,
	   const GraphWkPtr_t& graph, const NodeWkPtr_t& from,
	   const NodeWkPtr_t& to);

          virtual bool build (core::PathPtr_t& path, ConfigurationIn_t q1, ConfigurationIn_t q2, const core::WeighedDistance& d) const;

          virtual bool applyConstraints (ConfigurationIn_t qoffset, ConfigurationOut_t q) const;

          /// Return the inner waypoint.
          /// \param EdgeType is either Edge or WaypointEdge
          template <class EdgeType>
          boost::shared_ptr <EdgeType> waypoint () const;

          /// Print the object in a stream.
          virtual std::ostream& dotPrint (std::ostream& os, dot::DrawingAttributes da = dot::DrawingAttributes ()) const;

          /// Create inner waypoints.
          /// \param depth the number of waypoints between from() and to()
          ///              minus 1.
          /// \param bname basename used for naming.
          /// \note inner edges are named bname + "_e" + pos
          ///       inner nodes are named bname + "_n" + pos
          void createWaypoint (const unsigned int depth, const std::string& bname = "WaypointEdge");

          /// Get the node in which path after the waypoint is.
          NodePtr_t node () const;

        protected:
	  WaypointEdge (const std::string& name,
			const core::SteeringMethodPtr_t& steeringMethod) :
	    Edge (name, steeringMethod)
	    {
	    }
          /// Initialization of the object.
          void init (const WaypointEdgeWkPtr_t& weak, const GraphWkPtr_t& graph, const NodeWkPtr_t& from,
              const NodeWkPtr_t& to);

          /// Print the object in a stream.
          virtual std::ostream& print (std::ostream& os) const;

        private:
          typedef std::pair < EdgePtr_t, NodePtr_t > Waypoint;

          Waypoint waypoint_;
          mutable Configuration_t config_, result_;

          WaypointEdgeWkPtr_t wkPtr_;
      }; // class WaypointEdge

      /// Edge that find intersection of level set.
      class HPP_MANIPULATION_DLLAPI LevelSetEdge : public Edge
      {
        public:
          ~LevelSetEdge ();

          /// Create a new LevelSetEdge.
          static LevelSetEdgePtr_t create
	    (const std::string& name,
	     const core::SteeringMethodPtr_t& steeringMethod,
	     const GraphWkPtr_t& graph, const NodeWkPtr_t& from,
	     const NodeWkPtr_t& to);

          virtual bool applyConstraints (ConfigurationIn_t qoffset, ConfigurationOut_t q) const;

          virtual bool applyConstraints (core::NodePtr_t n_offset, ConfigurationOut_t q) const;

          void buildHistogram ();

          LeafHistogramPtr_t histogram () const;

          void insertConfigConstraint (const NumericalConstraintPtr_t& nm,
              const SizeIntervals_t& passiveDofs = SizeIntervals_t ());

          void insertConfigConstraint (const DifferentiableFunctionPtr_t function, const ComparisonTypePtr_t ineq) __attribute__ ((deprecated));

          void insertConfigConstraint (const LockedJointPtr_t lockedJoint);

          /// Print the object in a stream.
          virtual std::ostream& dotPrint (std::ostream& os, dot::DrawingAttributes da = dot::DrawingAttributes ()) const;

        protected:
          /// Initialization of the object.
          void init (const LevelSetEdgeWkPtr_t& weak, const GraphWkPtr_t& graph, const NodeWkPtr_t& from,
              const NodeWkPtr_t& to);

	  LevelSetEdge (const std::string& name,
			const core::SteeringMethodPtr_t& steeringMethod);

          /// Print the object in a stream.
          virtual std::ostream& print (std::ostream& os) const;

          /// Populate DrawingAttributes tooltip
          virtual void populateTooltip (dot::Tooltip& tp) const;

        private:
          typedef Cache < ConstraintSetPtr_t > Constraint_t;

          /// See pathConstraint member function.
          Constraint_t* extraConstraints_;
          virtual ConstraintSetPtr_t extraConfigConstraint () const;

          /// Extra NumericalConstraints_t
          NumericalConstraints_t extraNumericalConstraints_;
          IntervalsContainer_t extraPassiveDofs_;

          /// Extra LockedJoints_t
          LockedJoints_t extraLockedJoints_;

          /// This histogram will be used to find a good level set.
          LeafHistogramPtr_t hist_;

          LevelSetEdgeWkPtr_t wkPtr_;
      }; // class LevelSetEdge

      /// \}
    } // namespace graph
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_GRAPH_EDGE_HH
