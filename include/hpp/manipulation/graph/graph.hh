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

#ifndef HPP_MANIPULATION_GRAPH_GRAPH_HH
# define HPP_MANIPULATION_GRAPH_GRAPH_HH

# include "hpp/manipulation/config.hh"
# include "hpp/manipulation/fwd.hh"
# include "hpp/manipulation/graph/fwd.hh"
# include "hpp/manipulation/graph/graph-component.hh"

namespace hpp {
  namespace manipulation {
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
      /// - A Graph owns (i.e. has a shared pointer to) the NodeSelector s
      /// - A NodeSelector owns the Node s related to one gripper.
      /// - A Node owns its outgoing Edge s.
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

          /// Create and insert a NodeSelector inside the graph.
          NodeSelectorPtr_t createNodeSelector (const std::string& name);

          /// Set the nodeSelector
          /// \warning This should be done before adding nodes to the node
          /// selector otherwise the pointer to the parent graph will NOT be
          /// valid.
          void nodeSelector (NodeSelectorPtr_t ns);

          /// Get the nodeSelector
          NodeSelectorPtr_t nodeSelector () const
          {
            return nodeSelector_;
          }

          /// Returns the states of a configuration.
          NodePtr_t getNode (ConfigurationIn_t config) const;

          /// Returns the state of a roadmap node
          NodePtr_t getNode(RoadmapNodePtr_t node) const;

          /// Get possible edges between two nodes.
          Edges_t getEdges (const NodePtr_t& from, const NodePtr_t& to) const;

          /// Select randomly outgoing edge of the given node.
          EdgePtr_t chooseEdge(RoadmapNodePtr_t node) const;

          /// Constraint to project onto the Node.
          /// \param the Node_t on which to project.
          /// \return The initialized projector.
          ConstraintSetPtr_t configConstraint (const NodePtr_t& node);

          /// Constraint to project onto the same leaf as config.
          /// \param edges a list of edges defining the foliation.
          /// \return The constraint.
          ConstraintSetPtr_t configConstraint (const EdgePtr_t& edge);

	  /// Get error of a config with respect to a node constraint
	  ///
	  /// \param config Configuration,
	  /// \param node node containing the constraint to check config against
	  /// \retval error the error of the node constraint for the
	  ///         configuration
	  /// \return whether the configuration belongs to the node.
	  /// Call method core::ConstraintSet::isSatisfied for the node
	  /// constraints.
	  bool getConfigErrorForNode (ConfigurationIn_t config,
				      const NodePtr_t& node, vector_t& error);

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
				      const EdgePtr_t& edge, vector_t& error);

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
	     const EdgePtr_t& edge, vector_t& error);

          /// Constraint to project a path.
          /// \param edge a list of edges defining the foliation.
          /// \return The constraint.
          ConstraintSetPtr_t pathConstraint (const EdgePtr_t& edge);

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

	  /// Get the steering Method
	  const ProblemPtr_t& problem () const;

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

          /// Print the component in DOT language.
          virtual std::ostream& dotPrint (std::ostream& os, dot::DrawingAttributes da = dot::DrawingAttributes ()) const;

        protected:
          /// Initialization of the object.
          void init (const GraphWkPtr_t& weak, DevicePtr_t robot);

          /// Constructor
	  /// \param sm a steering method to create paths from edges
          Graph (const std::string& name, const ProblemPtr_t& problem) :
	    GraphComponent (name), problem_ (problem)
          {}

          /// Print the object in a stream.
          std::ostream& print (std::ostream& os) const;

        private:
          /// This list contains a node selector for each end-effector.
          NodeSelectorPtr_t nodeSelector_;

          /// A set of constraints that will always be used, for example
          /// stability constraints.
          ConstraintPtr_t constraints_;

          /// Keep a pointer to the composite robot.
          DevicePtr_t robot_;

          /// Weak pointer to itself.
          GraphWkPtr_t wkPtr_;

          /// Map of constraint sets (from Node).
          typedef std::map  < NodePtr_t, ConstraintSetPtr_t > MapFromNode;
          typedef std::pair < NodePtr_t, ConstraintSetPtr_t > PairNodeConstraints;
          MapFromNode constraintSetMapFromNode_;

          /// List of histograms
          Histograms_t hists_;

          /// Map of constraint sets (from Edge).
          typedef std::map  < EdgePtr_t, ConstraintSetPtr_t > MapFromEdge;
          typedef std::pair < EdgePtr_t, ConstraintSetPtr_t > PairEdgeConstraints;
          MapFromEdge cfgConstraintSetMapFromEdge_, pathConstraintSetMapFromEdge_;
	  ProblemPtr_t problem_;
          value_type errorThreshold_;
          size_type maxIterations_;
      }; // Class Graph

      /// \}
    } // namespace graph
  } // namespace manipulation

} // namespace hpp

#endif // HPP_MANIPULATION_GRAPH_GRAPH_HH
