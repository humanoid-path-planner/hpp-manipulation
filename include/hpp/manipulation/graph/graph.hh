// Copyright (c) 2014, LAAS-CNRS
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

#ifndef HPP_MANIPULATION_GRAPH_GRAPH_HH
# define HPP_MANIPULATION_GRAPH_GRAPH_HH

# include <tuple>
# include "hpp/manipulation/config.hh"
# include "hpp/manipulation/constraint-set.hh"
# include "hpp/manipulation/fwd.hh"
# include "hpp/manipulation/graph/fwd.hh"
# include "hpp/manipulation/graph/graph-component.hh"

namespace hpp {
  namespace manipulation {
    typedef constraints::ImplicitPtr_t ImplicitPtr_t;
    namespace graph {
      /// \addtogroup constraint_graph
      /// \{

      /// Description of the constraint graph.
      ///
      /// This class contains a graph representing a robot with several
      /// end-effectors.
      ///
      /// One must make sure not to create loop with shared pointers.
      /// To ensure that, the classes are defined as follow:
      /// - A Graph owns (i.e. has a shared pointer to) the StateSelector s
      /// - A StateSelector owns the Node s related to one gripper.
      /// - A State owns its outgoing Edge s.
      /// - An Edge does not own anything.
      class HPP_MANIPULATION_DLLAPI Graph : public GraphComponent
      {
        public:
          /// Create a new Graph.
	  ///
	  /// \param robot a manipulation robot
	  /// \param problem a pointer to the problem
	  static GraphPtr_t create(const std::string& name, DevicePtr_t robot,
				   const ProblemPtr_t& problem);

          GraphPtr_t self () const { return wkPtr_.lock(); }

          /// Create and insert a state selector inside the graph.
          StateSelectorPtr_t createStateSelector (const std::string& name);

          /// Set the state selector
          /// \warning This should be done before adding nodes to the node
          /// selector otherwise the pointer to the parent graph will NOT be
          /// valid.
          void stateSelector (StateSelectorPtr_t ns);

          /// Get the state selector
          StateSelectorPtr_t stateSelector () const
          {
            return stateSelector_;
          }

          /// Returns the state of a configuration.
          StatePtr_t getState (ConfigurationIn_t config) const;

          /// Returns the state of a roadmap node
          StatePtr_t getState (RoadmapNodePtr_t node) const;

          /// Get possible edges between two nodes.
          Edges_t getEdges (const StatePtr_t& from, const StatePtr_t& to) const;

          /// Select randomly outgoing edge of the given node.
          EdgePtr_t chooseEdge(RoadmapNodePtr_t node) const;

          /// Clear the vector of constraints and complements
          /// \sa registerConstraints
          void clearConstraintsAndComplement();

          /// Register a triple of constraints to be inserted in nodes and edges
          /// \param constraint a constraint (grasp of placement)
          /// \param complement the complement constraint
          /// \param both combination of the constraint and its complement. Both
          ///             constraints together corresponds to a full relative
          ///             transformation constraint
          /// When inserting constraints in transitions of the graph,
          /// in many cases, a constraint is associated to a state and
          /// the complement constraint is associated to the
          /// transition itself.  Registering those constraints
          /// priorly to graph construction makes possible to replace
          /// the constraint and its complement by the combination of
          /// both that is an explicit constraint.
          void registerConstraints (const ImplicitPtr_t& constraint,
                                    const ImplicitPtr_t& complement,
                                    const ImplicitPtr_t& both);

          /// Test whether two constraints are complement of one another
          ///
          /// \param constraint, complement two constraints to test
          /// \retval combinationOfBoth constraint corresponding to combining
          ///         constraint and complement if result is true,
          ///         unchanged otherwise.
          /// \return whether complement is the complement of constraint.
          /// Two constraints are complement of one another if and only if
          /// combined they constitute a complement relative transformation
          /// constraint. \sa Graph::registerConstraints
          /// \warning argument order matters.
          bool isComplement (const ImplicitPtr_t& constraint,
                             const ImplicitPtr_t& complement,
                             ImplicitPtr_t& combinationOfBoth) const;

          /// Return the vector of tuples as registered in registerConstraints
          /// \return a vector of tuples (c, c/complement, c/both) each of them
          ///         corresponding to a constraint, the complement constraint
          ///         and the combination of boths.
          const ConstraintsAndComplements_t& constraintsAndComplements () const;

          /// Constraint to project onto the Node.
          /// \param state the state on which to project.
          /// \return The initialized projector.
          ConstraintSetPtr_t configConstraint (const StatePtr_t& state) const;

          /// Constraints of path and target state of an edge
          /// \deprecated Use tagetConstraint instead.
          ConstraintSetPtr_t configConstraint (const EdgePtr_t& edge) const
            HPP_MANIPULATION_DEPRECATED;

          /// Constraints a configuration in target state should satisfy
          /// \param edge a transition
          /// \return The set of constraints a configuration lying in the
          ///         target state of the edge should satisfy. This set
          ///         includes the paths constraints of the edge.
          /// \sa Edge::targetConstraint.
          ConstraintSetPtr_t targetConstraint (const EdgePtr_t& edge) const;

	  /// Get error of a config with respect to a state constraint
	  ///
	  /// \param config Configuration,
	  /// \param state state containing the constraint to check config against
	  /// \retval error the error of the state constraint for the
	  ///         configuration
	  /// \return whether the configuration belongs to the state.
	  /// Call method core::ConstraintSet::isSatisfied for the state
	  /// constraints.
	  bool getConfigErrorForState (ConfigurationIn_t config,
				      const StatePtr_t& state, vector_t& error) const;

	  /// Get error of a config with respect to an edge constraint
	  ///
	  /// \param config Configuration,
	  /// \param edge edge containing the constraint to check config against
	  /// \retval error the error of the edge constraint for the
	  ///         configuration
	  /// \return whether the configuration can be a start point of a path
	  //          of the edge
	  /// Call core::ConfigProjector::rightHandSideFromConfig with
	  /// input configuration and method core::ConstraintSet::isSatisfied
	  /// for the edge constraint.
	  bool getConfigErrorForEdge (ConfigurationIn_t config,
				      const EdgePtr_t& edge, vector_t& error) const;

	  /// Get error of a config with respect to an edge foliation leaf
	  ///
	  /// \param leafConfig Configuration that determines the foliation leaf
	  /// \param config Configuration the error of which is computed
	  /// \retval error the error
	  /// \return whether config can be the end point of a path of the edge
	  ///         starting at leafConfig
	  /// Call methods core::ConfigProjector::rightHandSideFromConfig with
	  /// leafConfig and then core::ConstraintSet::isSatisfied with config.
	  /// on the edge constraints.
	  bool getConfigErrorForEdgeLeaf
	    (ConfigurationIn_t leafConfig, ConfigurationIn_t config,
	     const EdgePtr_t& edge, vector_t& error) const;

	  /// Get error of a config with respect to the target of an edge foliation leaf
	  ///
	  /// \param leafConfig Configuration that determines the foliation leaf
	  /// \param config Configuration the error of which is computed
	  /// \retval error the error
	  /// \return whether config can be the end point of a path of the edge
	  ///         starting at leafConfig
	  /// Call methods core::ConfigProjector::rightHandSideFromConfig with
	  /// leafConfig and then core::ConstraintSet::isSatisfied with config.
	  /// on the edge constraints.
	  bool getConfigErrorForEdgeTarget
	    (ConfigurationIn_t leafConfig, ConfigurationIn_t config,
	     const EdgePtr_t& edge, vector_t& error) const;

          /// Constraint to project a path.
          /// \param edge a list of edges defining the foliation.
          /// \return The constraint.
          ConstraintSetPtr_t pathConstraint (const EdgePtr_t& edge) const;

          /// Set maximal number of iterations
          void maxIterations (size_type iterations);

          /// Get maximal number of iterations in config projector
          size_type maxIterations () const;

          /// Set error threshold
          void errorThreshold (const value_type& threshold);

          /// Get error threshold in config projector
          value_type errorThreshold () const;

          /// Get the robot.
          const DevicePtr_t& robot () const;

	  /// Get the problem
	  const ProblemPtr_t& problem () const;

	  /// Set the problem
          void problem (const ProblemPtr_t& problem);

          /// Register an histogram representing a foliation
          void insertHistogram (const graph::HistogramPtr_t& hist)
          {
            hists_.push_back (hist);
          }

          /// Get the histograms
          const Histograms_t& histograms () const
          {
            return hists_;
          }

          /// Get the component by its ID.
          GraphComponentWkPtr_t get(std::size_t id) const;

          std::size_t nbComponents () const
          {
            return components_.size();
          }

          /// Print the component in DOT language.
          virtual std::ostream& dotPrint (std::ostream& os, dot::DrawingAttributes da = dot::DrawingAttributes ()) const;

          /// Initialize all components of the graph (edges and states)
          virtual void initialize ();

          /// Invalidate all states and edges of the graph
          virtual void invalidate();

        protected:
          /// Initialization of the object.
          void init (const GraphWkPtr_t& weak, DevicePtr_t robot);

          /// Constructor
	  /// \param sm a steering method to create paths from edges
          Graph (const std::string& name, const ProblemPtr_t& problem);

          /// Print the object in a stream.
          std::ostream& print (std::ostream& os) const;

        private:
          /// The list of elements
          GraphComponents_t& components ();

          /// Keep track of the created components
          GraphComponents_t components_;

          /// This list contains a state selector for each end-effector.
          StateSelectorPtr_t stateSelector_;

          /// A set of constraints that will always be used, for example
          /// stability constraints.
          ConstraintPtr_t constraints_;

          /// Keep a pointer to the composite robot.
          DevicePtr_t robot_;

          /// Weak pointer to itself.
          GraphWkPtr_t wkPtr_;

          /// Map of constraint sets (from State).
          typedef std::map  < StatePtr_t, ConstraintSetPtr_t > MapFromState;
          typedef std::pair < StatePtr_t, ConstraintSetPtr_t > PairStateConstraints;
          MapFromState constraintSetMapFromState_;

          /// List of histograms
          Histograms_t hists_;

          /// Map of constraint sets (from Edge).
          typedef std::map  < EdgePtr_t, ConstraintSetPtr_t > MapFromEdge;
          typedef std::pair < EdgePtr_t, ConstraintSetPtr_t > PairEdgeConstraints;
          MapFromEdge cfgConstraintSetMapFromEdge_, pathConstraintSetMapFromEdge_;
	  ProblemPtr_t problem_;
          value_type errorThreshold_;
          size_type maxIterations_;

          ConstraintsAndComplements_t constraintsAndComplements_;
          friend class GraphComponent;
      }; // Class Graph

      /// \}
    } // namespace graph
  } // namespace manipulation

} // namespace hpp

BOOST_CLASS_EXPORT_KEY(hpp::manipulation::graph::Graph)

#endif // HPP_MANIPULATION_GRAPH_GRAPH_HH
