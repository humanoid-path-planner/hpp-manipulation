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

#include <hpp/util/assertion.hh>

#include "hpp/manipulation/robot.hh"
#include "hpp/manipulation/graph/node-selector.hh"
#include "hpp/manipulation/graph/node.hh"
#include "hpp/manipulation/graph/edge.hh"
#include "hpp/manipulation/graph/graph.hh"

namespace hpp {
  namespace manipulation {
    namespace graph {
      GraphPtr_t Graph::create(RobotPtr_t robot)
      {
        Graph* ptr = new Graph;
        GraphPtr_t shPtr (ptr);
        ptr->init (shPtr, robot);
        return shPtr;
      }

      void Graph::init (const GraphWkPtr_t& weak, RobotPtr_t robot)
      {
        GraphComponent::init (weak);
        robot_ = robot;
        wkPtr_ = weak;
      }

      NodeSelectorPtr_t Graph::createNodeSelector()
      {
        NodeSelectorPtr_t newNodeSelector = NodeSelector::create();
        nodeSelectors_.push_back(newNodeSelector);
        newNodeSelector->parentGraph (wkPtr_);
        return newNodeSelector;
      }

      void Graph::maxIterations (size_type iterations)
      {
        maxIterations_ = iterations;
      }

      size_type Graph::maxIterations () const
      {
        return maxIterations_;
      }

      void Graph::errorThreshold (const value_type& threshold)
      {
        errorThreshold_ = threshold;
      }

      value_type Graph::errorThreshold () const
      {
        return errorThreshold_;
      }

      const RobotPtr_t& Graph::robot () const
      {
        return robot_;
      }

      Nodes_t Graph::getNode(const Configuration_t config)
      {
        Nodes_t nodes;
        for (NodeSelectors_t::iterator it = nodeSelectors_.begin();
            it != nodeSelectors_.end(); it++)
          nodes.push_back( (*it)->getNode(config) );
        return nodes;
      }

      Edges_t Graph::chooseEdge(const Nodes_t& nodes)
      {
        Edges_t edges;
        for (Nodes_t::const_iterator it = nodes.begin();
            it != nodes.end(); it++)
          edges.push_back( (*it)->nodeSelector().lock()->chooseEdge(*it) );
        return edges;
      }

      NodeSelectorPtr_t Graph::getNodeSelectorByName (const std::string& name)
      {
        for (NodeSelectors_t::iterator it = nodeSelectors_.begin();
            it != nodeSelectors_.end(); it++) {
          if (name == (*it)->name())
            return *it;
        }
        return NodeSelectorPtr_t();
      }

      ConstraintPtr_t Graph::configConstraint (const Nodes_t& nodes)
      {
        ConstraintSetPtr_t constraint = ConstraintSet::create (robot (), name ());

        ConfigProjectorPtr_t proj = ConfigProjector::create(robot(), name (), errorThreshold(), maxIterations());
        insertListIn <DifferentiableFunctions_t> (numericalConstraints (), proj);
        for (Nodes_t::const_iterator it = nodes.begin();
            it != nodes.end(); it++)
          insertListIn <DifferentiableFunctions_t> ((*it)->numericalConstraints (), proj);
        constraint->addConstraint (HPP_DYNAMIC_PTR_CAST(Constraint, proj));

        insertListIn <LockedDofs_t> (lockedDofConstraints (), constraint);
        for (Nodes_t::const_iterator it = nodes.begin();
            it != nodes.end(); it++)
          insertListIn <LockedDofs_t> ((*it)->lockedDofConstraints (), constraint);

        return constraint;
      }

      ConstraintPtr_t Graph::configConstraint (const Edges_t& edges, ConfigurationIn_t config)
      {
        ConstraintSetPtr_t constraint = ConstraintSet::create (robot (), name ());

        ConfigProjectorPtr_t proj = ConfigProjector::create(robot(), name (), errorThreshold(), maxIterations());
        insertListIn <DifferentiableFunctions_t> (numericalConstraints (), proj);
        for (Edges_t::const_iterator it = edges.begin();
            it != edges.end(); it++) {
          insertListIn <DifferentiableFunctions_t> ((*it)->numericalConstraints (), proj);
          insertListIn <DifferentiableFunctions_t> ((*it)->to()->numericalConstraints (), proj);
        }
        constraint->addConstraint (HPP_DYNAMIC_PTR_CAST(Constraint, proj));

        insertListIn <LockedDofs_t> (lockedDofConstraints (), constraint);
        for (Edges_t::const_iterator it = edges.begin();
            it != edges.end(); it++) {
          insertListIn <LockedDofs_t> ((*it)->lockedDofConstraints (), constraint);
          insertListIn <LockedDofs_t> ((*it)->to()->lockedDofConstraints(), constraint);
        }

        constraint->offsetFromConfig (config);
        return constraint;
      }

      ConstraintPtr_t Graph::pathConstraint (const Edges_t& edges, ConfigurationIn_t config)
      {
        ConstraintSetPtr_t constraint = ConstraintSet::create (robot (), name ());

        ConfigProjectorPtr_t proj = ConfigProjector::create(robot(), name (), errorThreshold(), maxIterations());
        insertListIn <DifferentiableFunctions_t> (numericalConstraints (), proj);
        for (Edges_t::const_iterator it = edges.begin();
            it != edges.end(); it++)
          insertListIn <DifferentiableFunctions_t> ((*it)->numericalConstraints (), proj);
        constraint->addConstraint (HPP_DYNAMIC_PTR_CAST(Constraint, proj));

        insertListIn <LockedDofs_t> (lockedDofConstraints (), constraint);
        for (Edges_t::const_iterator it = edges.begin();
            it != edges.end(); it++)
          insertListIn <LockedDofs_t> ((*it)->lockedDofConstraints (), constraint);

        constraint->offsetFromConfig (config);
        return constraint;
      }

      std::ostream& Graph::print (std::ostream& os) const
      {
        GraphComponent::print (os) << std::endl;
        for (NodeSelectors_t::const_iterator it = nodeSelectors_.begin();
            it != nodeSelectors_.end(); it++)
          os << *(*it);
        return os;
      }
    } // namespace graph
  } // namespace manipulation

} // namespace hpp
