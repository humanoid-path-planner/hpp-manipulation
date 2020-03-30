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
#include <hpp/core/steering-method.hh>
#include <hpp/core/relative-motion.hh>
#include <hpp/core/path.hh>

#include "hpp/manipulation/config.hh"
#include "hpp/manipulation/fwd.hh"
#include "hpp/manipulation/graph/graph.hh"

namespace hpp {
  namespace manipulation {
    namespace graph {
      /// \addtogroup constraint_graph
      /// \{

      /// Transition between two states of a constraint graph
      ///
      /// An edge stores two types of constraints.

      /// \li <b> Path constraints </b> should be safisfied by paths belonging
      /// to the edge. Along any path, the right hand side of the
      /// constraint is constant, but can differ between paths. For
      /// instance if an edge represents a transit path of a robot
      /// that can grasp an object, the right hand side of the
      /// constraint represents the position of the object. Along any
      /// transit path, the object does not move, but for different paths
      /// the object can be at different positions.
      /// \sa method pathConstraint.
      /// \li <b> Configuration constraints </b> are constraints that
      /// configurations in the destination state should satisfy and
      /// the constraints that paths should satisfy. For instance, if
      /// the edge links a state where the robot does not hold the
      /// object to a state where the robot holds the object, the
      /// configuration constraints represent a fixed relative
      /// position of the object with respect to the gripper and a
      /// stable position of the object. Configuration constraints are
      /// necessary to generate a configuration in the destination
      /// state of the edge that is reachable from a given
      /// configuration in the start state by an admissible path.
      class HPP_MANIPULATION_DLLAPI Edge : public GraphComponent
      {
        public:
          typedef core::RelativeMotion RelativeMotion;

          /// Destructor
          virtual ~Edge ();

          /// Create a new empty Edge.
          static EdgePtr_t create
	    (const std::string& name,
	     const GraphWkPtr_t& graph,
	     const StateWkPtr_t& from,
	     const StateWkPtr_t& to);

	  /// Generate a reachable configuration in the target state
	  ///
	  /// \param nStart node containing the configuration defining the right
	  ///        hand side of the edge constraint,
	  /// \param[in,out] q input configuration used to initialize the
          ///                numerical solver and output configuration lying
          ///                in the target state and reachable along the edge
          ///                from nnear
          /// \deprecated Use generateTargetConfig instead.
          virtual bool applyConstraints (core::NodePtr_t nStart,
                                         ConfigurationOut_t q) const
            HPP_MANIPULATION_DEPRECATED;

	  /// Generate a reachable configuration in the target state
	  ///
	  /// \param qStart node containing the configuration defining the right
	  ///        hand side of the edge path constraint,
	  /// \param[in,out] q input configuration used to initialize the
          ///                numerical solver and output configuration lying
          ///                in the target state and reachable along the edge
          ///                from nnear.
          /// \deprecated Use generateTargetConfig instead.
          virtual bool applyConstraints (ConfigurationIn_t qStart,
                                         ConfigurationOut_t q) const
            HPP_MANIPULATION_DEPRECATED;

	  /// Generate a reachable configuration in the target state
	  ///
	  /// \param nStart node containing the configuration defining the right
	  ///        hand side of the edge path constraint,
	  /// \param[in,out] q input configuration used to initialize the
          ///                numerical solver and output configuration lying
          ///                in the target state and reachable along the edge
          ///                from nStart.
          virtual bool generateTargetConfig(core::NodePtr_t nStart,
                                            ConfigurationOut_t q) const;

	  /// Generate a reachable configuration in the target state
	  ///
	  /// \param qStart node containing the configuration defining the right
	  ///        hand side of the edge path constraint,
	  /// \param[in,out] q input configuration used to initialize the
          ///                numerical solver and output configuration lying
          ///                in the target state and reachable along the edge
          ///                from nnear.
          virtual bool generateTargetConfig (ConfigurationIn_t qStart,
                                             ConfigurationOut_t q) const;

          virtual bool canConnect (ConfigurationIn_t q1, ConfigurationIn_t q2) const;

          virtual bool build (core::PathPtr_t& path, ConfigurationIn_t q1,
              ConfigurationIn_t q2) const;

          /// Get the destination
          StatePtr_t stateTo () const;

          /// Get the origin
          StatePtr_t stateFrom () const;

          /// Get the state in which path is.
          StatePtr_t state () const
          {
            return state_.lock();
          }

          void state (StatePtr_t state)
          {
            state_ = state;
          }

	  /// Get steering method associated to the edge.
	  const core::SteeringMethodPtr_t& steeringMethod () const
	  {
	    return steeringMethod_;
	  }

	  /// Get path validation associated to the edge.
	  const core::PathValidationPtr_t& pathValidation () const
	  {
	    return pathValidation_;
	  }

          const RelativeMotion::matrix_type& relativeMotion () const
          {
            return relMotion_;
          }

          /// Update the relative motion matrix
          void relativeMotion(const RelativeMotion::matrix_type & m);

          /// Set Security margin for a pair of joints
          ///
          /// \param row index of joint1 + 1 in robot,
          /// \param col index of joint2 + 1 in robot,
          /// \param margin security margin for collision checking between
          ///        those joints.
          ///
          /// \note set value to matrix [row, col] and matrix [col, row].
          void securityMarginForPair(const size_type& row, const size_type& col,
                                     const value_type& margin);

          /// Accessor to the security margin.
          const matrix_t& securityMargins() const
          {
            return securityMargins_;
          }

          /// Get direction of the path compare to the edge
          /// \return true is reverse
          virtual bool direction (const core::PathPtr_t& path) const;

          /// Populate a ConfigProjector with constraints required to generate
          /// a path at the intersection of two edges.
          virtual bool intersectionConstraint (const EdgePtr_t& other,
              ConfigProjectorPtr_t projector) const;

          /// Print the object in a stream.
          virtual std::ostream& dotPrint (std::ostream& os, dot::DrawingAttributes da = dot::DrawingAttributes ()) const;

          /// Constraint of the destination state and of the path
          /// \deprecated Use targetConstraint instead
          ConstraintSetPtr_t configConstraint() const
            HPP_MANIPULATION_DEPRECATED;

          /// Constraint of the destination state and of the path
          ConstraintSetPtr_t targetConstraint() const;

          void setShort (bool isShort) {
            isShort_ = isShort;
          }

          bool isShort () const {
            return isShort_;
          }
	   /// Constraint to project a path.
          /// \return The initialized constraint.
          ConstraintSetPtr_t pathConstraint() const;

        protected:
          /// Initialization of the object.
          void init (const EdgeWkPtr_t& weak, const GraphWkPtr_t& graph, const StateWkPtr_t& from,
              const StateWkPtr_t& to);

          /// Constructor
          Edge (const std::string& name);

          virtual ConstraintSetPtr_t buildConfigConstraint()
            HPP_MANIPULATION_DEPRECATED;

          /// Build path and target state constraint set.
          virtual ConstraintSetPtr_t buildTargetConstraint();

          /// Build path constraints
          virtual ConstraintSetPtr_t buildPathConstraint();

          virtual void initialize ();

          /// Print the object in a stream.
          virtual std::ostream& print (std::ostream& os) const;

          bool isShort_;

        private:
          /// See pathConstraint member function.
          ConstraintSetPtr_t pathConstraints_;

          /// Constraint ensuring that a q_proj will be in to_ and in the
          /// same leaf of to_ as the configuration used for initialization.
          ConstraintSetPtr_t targetConstraints_;

          /// The two ends of the transition.
          StateWkPtr_t from_, to_;

          /// True if this path is in state from, False if in state to
          StateWkPtr_t state_;

	  /// Steering method used to create paths associated to the edge
          core::SteeringMethodPtr_t steeringMethod_;

	  /// Path validation associated to the edge
          mutable RelativeMotion::matrix_type relMotion_;
          /// matrix of security margins for collision checking between joints
          matrix_t securityMargins_;

          core::PathValidationPtr_t pathValidation_;

          /// Weak pointer to itself.
          EdgeWkPtr_t wkPtr_;

          friend class Graph;
      }; // class Edge

      /// Edge with intermediate waypoint states.
      ///
      /// This class implements a transition from one state to another state
      /// with intermediate states in-between. This feature is particularly
      /// interesting when manipulating objects. Between a state where a gripper
      /// does not grasp an object and a state where the same gripper grasps
      /// the object, it is useful to add an intermediate state where the
      /// gripper is close to the object.
      ///
      /// Waypoints are handled recursively, i.e.\ class WaypointEdge
      /// contains only a State and an Edge, the second Edge being
      /// itself. In this package, the State in a WaypointEdge is
      /// semantically different from other State because it does not
      /// correspond to a state with different manipulation rules. It
      /// has the same rules as another State (either Edge::stateFrom() or
      /// Edge::stateTo()).
      ///
      /// Semantically, a waypoint State is fully part of the WaypointEdge.
      /// When a corresponding path reaches it, no planning is required to know
      /// what to do next. To the contrary, when a path reaches
      /// Edge::stateFrom() or Edge::stateTo(), there may be several
      /// possibilities.
      ///
      /// \note
      ///   Implementation details: let's say, between the two states \f$N_f\f$
      ///   and \f$N_t\f$, two waypoints are required:
      ///   \f$ N_f \xrightarrow{e_0} n_0 \xrightarrow{e_1} n_1
      ///   \xrightarrow{e_2} N_t\f$.
      ///   The WaypointEdge contains:
      ///   \li from: \f$N_f\f$,
      ///   \li to: \f$N_t\f$,
      ///   \li states: \f$(n_0, n_1)\f$
      ///   \li transitions: \f$(e_0, e_1, e_2)\f$
      ///   \li constraints: any calls to the constraints throw,
      class HPP_MANIPULATION_DLLAPI WaypointEdge : public Edge
      {
        public:
          /// Create a new WaypointEdge.
	static WaypointEdgePtr_t create
	  (const std::string& name,
	   const GraphWkPtr_t& graph, const StateWkPtr_t& from,
	   const StateWkPtr_t& to);

          virtual bool canConnect (ConfigurationIn_t q1, ConfigurationIn_t q2) const;

          virtual bool build (core::PathPtr_t& path, ConfigurationIn_t q1, ConfigurationIn_t q2) const;


	  /// Generate a reachable configuration in the target state
	  ///
	  /// \param qStart node containing the configuration defining the right
	  ///        hand side of the edge path constraint,
	  /// \param[in,out] q input configuration used to initialize the
          ///                numerical solver and output configuration lying
          ///                in the target state and reachable along the edge
          ///                from nnear.
          /// deprecated Used generateTargetConfig instead.
          virtual bool applyConstraints (ConfigurationIn_t qStart,
                                         ConfigurationOut_t q) const
            HPP_MANIPULATION_DEPRECATED;

	  /// Generate a reachable configuration in the target state
	  ///
	  /// \param qStart node containing the configuration defining the right
	  ///        hand side of the edge path constraint,
	  /// \param[in,out] q input configuration used to initialize the
          ///                numerical solver and output configuration lying
          ///                in the target state and reachable along the edge
          ///                from nnear.
          virtual bool generateTargetConfig (ConfigurationIn_t qStart,
                                             ConfigurationOut_t q) const;

          /// Return the index-th edge.
          const EdgePtr_t& waypoint (const std::size_t index) const;

          /// Print the object in a stream.
          virtual std::ostream& dotPrint (std::ostream& os, dot::DrawingAttributes da = dot::DrawingAttributes ()) const;

          /// Set the number of waypoints
          void nbWaypoints (const size_type number);

          std::size_t nbWaypoints () const
          {
            return edges_.size () - 1;
          }

          /// Set waypoint index with wEdge and wTo.
          /// \param wTo is the destination state of wEdge
          void setWaypoint (const std::size_t index, const EdgePtr_t wEdge, const StatePtr_t wTo);

        protected:
	  WaypointEdge (const std::string& name) :
	    Edge (name),
            lastSucceeded_ (false)
	    {
	    }
          /// Initialization of the object.
          void init (const WaypointEdgeWkPtr_t& weak, const GraphWkPtr_t& graph, const StateWkPtr_t& from,
              const StateWkPtr_t& to);

          /// Print the object in a stream.
          virtual std::ostream& print (std::ostream& os) const;

        private:
          Edges_t edges_;
          States_t states_;

          mutable matrix_t configs_;
          mutable bool lastSucceeded_;

          WaypointEdgeWkPtr_t wkPtr_;
      }; // class WaypointEdge

      /// Edge that handles crossed foliations
      ///
      /// Let us consider the following simple constraint graph
      /// corresponding to a robot grasping an object with one gripper.
      ///
      /// \image html constraint-graph.png "Simple constraint graph corresponding to a robot grasping an object."
      ///
      /// In order to disambiguate, we assume here that
      /// \li transition <b>Grasp object</b> is in \b Placement state,
      /// \li transition <b>Release object</b> is in \b Grasp state.
      ///
      /// If state \b Placement is defined by the object lying on a planar
      /// polygonal surface, then
      /// \li state \b Placement,
      /// \li transition \b Transit, and
      /// \li transition <b>Grasp object</b>
      ///
      /// are all constrained in a foliated manifold parameterized by the
      /// position of the box on the surface.
      ///
      /// Likewise, if the object is cylindrical the grasp may have a degree
      /// of freedom corresponding to the angle around z-axis of the gripper
      /// with respect to the object. See classes
      /// \link hpp::manipulation::Handle Handle\endlink and
      /// \link hpp::pinocchio::Gripper Gripper\endlink for details.
      /// In this latter case,
      /// \li state \b Grasp,
      /// \li transition \b Transfer, and
      /// \li transition <b>Release object</b>
      ///
      /// are all constrained in a foliated manifold parameterized by the
      /// angle around z-axis of the gripper with respect to the object.
      ///
      /// Let us denote
      /// \li \c grasp the numerical constraint defining state \b Grasp,
      /// \li \c placement the numerical constraint defining state \b Placement,
      /// \li \c grasp_comp the parameterized constraint defining a leaf
      ///     of \c Transfer (the angle between the gripper and the
      ///     object),
      /// \li \c placement_comp the parameterized constraint defining a leaf
      ///     of \b Placement (the position of the object on the contact
      ///     surface).
      ///
      /// As explained in <a
      /// href="https://hal.archives-ouvertes.fr/hal-01358767">this
      /// paper </a>, we are in the crossed foliation case and manipulation RRT
      /// will never be able to connect trees expanding in different leaves of
      /// the foliation.
      ///
      /// This class solves this issue in the following way by creating an
      /// instance of LevelSetEdge between \b Placement and \b Grasp.
      ///
      /// When extending a configuration \f$\mathbf{q}_{start}\f$ in state
      /// \b Placement, this transition will produce a target configuration
      /// (method \link LevelSetEdge::generateTargetConfig generateTargetConfig)
      /// \endlink as follows.
      ///
      /// \li pick a random configuration \f$\mathbf{q}_rand\f$, in the edge
      /// histogram (see method \link LevelSetEdge::histogram histogram\endlink)
      /// \li compute right hand side of \c grasp_comp with
      ///     \f$\mathbf{q}_{rand}\f$,
      /// \li compute right hand side of \c placement_comp with
      ///     \f$\mathbf{q}_{start}\f$,
      /// \li solve (\c grasp, \c placement, \c placement_comp, \c grasp_comp)
      ///     using input configuration \f$\mathbf{q}\f$. Note that the
      /// parent method Edge::generateTargetConfig does the same without
      /// adding \c grasp_comp.
      ///
      /// The constraints parameterizing the target state foliation
      /// (\c graps_comp in our example) are passed to class instances
      /// using method \link LevelSetEdge::insertParamConstraint
      /// insertParamConstraint\endlink.
      class HPP_MANIPULATION_DLLAPI LevelSetEdge : public Edge
      {
        public:
          virtual ~LevelSetEdge ();

          /// Create a new LevelSetEdge.
          static LevelSetEdgePtr_t create
	    (const std::string& name,
	     const GraphWkPtr_t& graph, const StateWkPtr_t& from,
	     const StateWkPtr_t& to);

	  /// Generate a reachable configuration in the target state
	  ///
	  /// \param nStart node containing the configuration defining the right
	  ///        hand side of the edge constraint,
	  /// \param[in,out] q input configuration used to initialize the
          ///                numerical solver and output configuration lying
          ///                in the target state and reachable along the edge
          ///                from nnear
          /// \deprecated Use generateTargetConfig instead.
          virtual bool applyConstraints (core::NodePtr_t nStart,
                                         ConfigurationOut_t q) const
            HPP_MANIPULATION_DEPRECATED;

	  /// Generate a reachable configuration in the target state
	  ///
	  /// \param qStart node containing the configuration defining the right
	  ///        hand side of the edge path constraint,
	  /// \param[in,out] q input configuration used to initialize the
          ///                numerical solver and output configuration lying
          ///                in the target state and reachable along the edge
          ///                from nnear.
          /// \deprecated Use generateTargetConfig instead.
          virtual bool applyConstraints (ConfigurationIn_t qStart,
                                         ConfigurationOut_t q) const
            HPP_MANIPULATION_DEPRECATED;

	  /// Generate a reachable configuration in the target state
	  ///
	  /// \param nStart node containing the configuration defining the right
	  ///        hand side of the edge path constraint,
	  /// \param[in,out] q input configuration used to initialize the
          ///                numerical solver and output configuration lying
          ///                in the target state and reachable along the edge
          ///                from nStart.
          virtual bool generateTargetConfig(core::NodePtr_t nStart,
                                            ConfigurationOut_t q) const;

	  /// Generate a reachable configuration in the target state
	  ///
	  /// \param qStart node containing the configuration defining the right
	  ///        hand side of the edge path constraint,
	  /// \param[in,out] q input configuration used to initialize the
          ///                numerical solver and output configuration lying
          ///                in the target state and reachable along the edge
          ///                from nnear.
          virtual bool generateTargetConfig (ConfigurationIn_t qStart,
                                             ConfigurationOut_t q) const;

          /// \deprecated Use buildTargetConstraint instead
          virtual ConstraintSetPtr_t buildConfigConstraint()
            HPP_MANIPULATION_DEPRECATED;

          /// Build path and target state constraints
          virtual ConstraintSetPtr_t buildTargetConstraint();

          /// Build the histogram
          /// \sa LevelSetEdge::histogram.
          void buildHistogram ();

          /// Return pointer on histogram of the edge
          ///
          /// The edge histogram is a container of configurations defined by
          /// a set of constraints called the <b>condition constraints</b>
          /// that a configuration should satisfy to be inserted in the
          /// histogram.
          ///
          /// The histogram is passed to the Roadmap via the graph (method
          /// Graph::insertHistogram). The roadmap then populates the histogram
          /// with all new configurations satisfying the condition constraints.
          ///
          /// The condition constraints should therefore be the constraints of
          /// the target state of the level set edge.
          ///
          /// \sa LevelSetEdge::insertConditionConstraint
          LeafHistogramPtr_t histogram () const;

          /// \name Foliation definition
          /// \{

          /// Insert a constraints parameterizing the target state foliation
          /// \param nm the numerical constraint,
          /// \param passiveDofs the passive degrees of freedom of the
          ///        constraint.
          void insertParamConstraint (const ImplicitPtr_t& nm,
              const segments_t& passiveDofs = segments_t ());

          /// Insert a condition constraint
          /// \sa LevelSetEdge::histogram
          void insertConditionConstraint (const ImplicitPtr_t& nm,
              const segments_t& passiveDofs = segments_t ());

          /// \}

          /// Print the object in a stream.
          virtual std::ostream& dotPrint (std::ostream& os, dot::DrawingAttributes da = dot::DrawingAttributes ()) const;

        protected:
          /// Initialization of the object.
          void init (const LevelSetEdgeWkPtr_t& weak, const GraphWkPtr_t& graph, const StateWkPtr_t& from,
              const StateWkPtr_t& to);

	  LevelSetEdge (const std::string& name);

          /// Print the object in a stream.
          virtual std::ostream& print (std::ostream& os) const;

          /// Populate DrawingAttributes tooltip
          virtual void populateTooltip (dot::Tooltip& tp) const;

          virtual void initialize ();

        private:
          bool applyConstraintsWithOffset (ConfigurationIn_t qoffset,
              ConfigurationIn_t qlevelset, ConfigurationOut_t q) const;

          // Parametrizer
          // NumericalConstraints_t
          NumericalConstraints_t paramNumericalConstraints_;
          IntervalsContainer_t paramPassiveDofs_;

          // Condition
          // NumericalConstraints_t
          NumericalConstraints_t condNumericalConstraints_;
          IntervalsContainer_t condPassiveDofs_;

          /// This histogram will be used to find a good level set.
          LeafHistogramPtr_t hist_;

          LevelSetEdgeWkPtr_t wkPtr_;
      }; // class LevelSetEdge

      /// \}
    } // namespace graph
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_GRAPH_EDGE_HH
