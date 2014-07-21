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

# include <string>

# include "hpp/manipulation/robot.hh"

# include "hpp/manipulation/config.hh"
# include "hpp/manipulation/fwd.hh"
# include "hpp/manipulation/graph/fwd.hh"

namespace hpp {
  namespace manipulation {
    namespace graph {
      /// Define common methods of the graph components.
      class HPP_MANIPULATION_LOCAL GraphComponent
      {
        public:
          /// Get the component name.
          const std::string& name() const
          {
            return name_;
          }

          /// Set the component name.
          void name(const std::string& name)
          {
            name_ = name;
          }

        private:
          std::string name_;
      };


      /// Description of the constraint graph
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
          static GraphPtr_t create(RobotPtr_t robot);

          /// Create and insert a NodeSelector inside the graph.
          NodeSelectorPtr_t createNodeSelector();

          /// Returns the states of a configuration.
          virtual Nodes_t getNode(const Configuration_t config);

          /// Select randomly outgoing edges of the given nodes.
          virtual Edges_t chooseEdge(const Nodes_t& node);

        protected:
          /// Initialization of the object.
          void init (const GraphWkPtr_t& weak, RobotPtr_t robot);

          /// Constructor
          Graph ()
          {}

        private:
          /// This list contains a node selector for each end-effector.
          NodeSelectors_t nodeSelectors_;

          /// A set of constraints that will always be used, for example
          /// stability constraints.
          ConstraintPtr_t constraints_;

          /// Keep a pointer to the composite robot.
          RobotPtr_t robot_;

          /// Weak pointer to itself.
          GraphWkPtr_t wkPtr_;
      }; // Class Graph
    } // namespace graph
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_GRAPH_GRAPH_HH
