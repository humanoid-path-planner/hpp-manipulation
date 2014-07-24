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
# include <ostream>
# include <hpp/util/assertion.hh>
# include <hpp/util/exception.hh>

# include "hpp/manipulation/robot.hh"

# include "hpp/manipulation/config.hh"
# include "hpp/manipulation/fwd.hh"
# include "hpp/manipulation/graph/fwd.hh"

namespace hpp {
  namespace manipulation {
    namespace graph {
      HPP_MAKE_EXCEPTION ( HPP_MANIPULATION_DLLAPI, Bad_function_call );

      /// Define common methods of the graph components.
      class HPP_MANIPULATION_DLLAPI GraphComponent
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

          /// Get the component by its ID. The validity of the GraphComponent
          /// is not checked.
          static GraphComponentWkPtr_t get(int id)
            throw (std::out_of_range)
          {
# ifdef HPP_DEBUG
            if (id < 0 || id >= (int)components.size())
              throw std::out_of_range ("ID out of range.");
# endif // HPP_DEBUG
            return components[id];
          };

          /// Return the component id.
          int id () const
          {
            return id_;
          }

          /// Print the object in a stream.
          virtual std::ostream& print (std::ostream& os) const
          {
            os << id () << " : " << name ();
            return os;
          }

          /// Add core::DifferentiableFunction to the component.
          virtual void addNumericalConstraint (const DifferentiableFunctionPtr_t& function)
          {
            numericalConstraints_.push_back(function);
          }

          /// Add core::LockedDof constraint to the component.
          virtual void addLockedDofConstraint (const LockedDofPtr_t& constraint)
          {
            lockedDofConstraints_.push_back (constraint);
          }

          /// Get a reference to the DifferentiableFunctions_t
          const DifferentiableFunctions_t& numericalConstraints() const
          {
            return numericalConstraints_;
          }

          /// Get a reference to the LockedDofs_t
          const LockedDofs_t& lockedDofConstraints () const
          {
            return lockedDofConstraints_;
          }

          /// Set the parent graph.
          void parentGraph(const GraphWkPtr_t& parent)
          {
            graph_ = parent;
          }

        protected:
          /// Initialize the component
          void init (const GraphComponentWkPtr_t& weak)
          {
            wkPtr_ = weak;
            id_ = components.size();
            components.push_back (wkPtr_);
          }

          GraphComponent() : id_(-1)
          {}

          /// Stores the numerical constraints.
          DifferentiableFunctions_t numericalConstraints_;
          /// List of LockedDof constraints
          LockedDofs_t lockedDofConstraints_;
          /// A weak pointer to the parent graph.
          GraphWkPtr_t graph_;

        private:
          /// Keep track of the created components in order to retrieve them
          /// easily.
          static std::vector < GraphComponentWkPtr_t > components;

          /// Name of the component.
          std::string name_;
          /// Weak pointer to itself.
          GraphComponentWkPtr_t wkPtr_;
          /// ID of the component (index in components vector).
          int id_;
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

          /// Return the NodeSelector with the given name if any,
          /// NULL pointer if not found.
          NodeSelectorPtr_t getNodeSelectorByName (const std::string& name);

          /// Print the object in a stream.
          std::ostream& print (std::ostream& os) const;


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

  HPP_MANIPULATION_DLLAPI std::ostream& operator<< (std::ostream& os,
      const hpp::manipulation::graph::GraphComponent& graphComp);

} // namespace hpp

#endif // HPP_MANIPULATION_GRAPH_GRAPH_HH
