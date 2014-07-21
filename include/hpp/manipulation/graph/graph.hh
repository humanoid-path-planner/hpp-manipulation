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

# include "hpp/manipulation/robot.hh"
# include "hpp/manipulation/graph/node-selector.hh"

# include "hpp/manipulation/fwd.hh"
# include "hpp/manipulation/graph/fwd.hh"

namespace hpp {
  namespace manipulation {
    namespace graph {
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
      class HPP_MANIPULATION_DLLAPI Graph
      {
        public:
          /// Create a new Graph.
          static GraphPtr_t create(RobotPtr_t robot)
          {
            Graph* ptr = new Graph;
            GraphPtr_t shPtr (ptr);
            ptr->init (shPtr, robot);
            return shPtr;
          }

          /// Create and insert a NodeSelector inside the graph.
          NodeSelectorPtr_t createNodeSelector()
          {
            NodeSelectorPtr_t newNodeSelector = NodeSelector::create();
            nodeSelectors_.push_back(newNodeSelector);
            return newNodeSelector;
          }

          /// Returns the states of a configuration.
          virtual Nodes_t getNode(const Configuration_t config) const;

          /// Select randomly outgoing edges of the given nodes.
          virtual Edges_t chooseEdge(const Nodes_t& node) const;

        protected:
          /// Initialization of the object.
          void init (const GraphWkPtr_t& weak, RobotPtr_t robot)
          {
            robot_ = robot;
            wkPtr_ = weak;
          }

          /// Constructor
          Graph ()
          {}

        private:
          /// This list contains a node selector for each end-effector.
          std::vector < NodeSelectorPtr_t > nodeSelectors_;

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
