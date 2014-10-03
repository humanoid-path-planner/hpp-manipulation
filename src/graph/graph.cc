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
      static std::string toString (const NodePtr_t& n) {
        std::string nodesStr = "(" + n->name () + ")";
        return nodesStr;
      }

      static std::string toString (const EdgePtr_t& e) {
        std::string edgesStr = "(" + e->name () + ")";
        return edgesStr;
      }

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
        nodeSelector_ = NodeSelector::create();
        nodeSelector_->parentGraph (wkPtr_);
        return nodeSelector_;
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

      NodePtr_t Graph::getNode (const Configuration_t config) const
      {
        return nodeSelector_->getNode (config);
      }

      Edges_t Graph::getEdges (const NodePtr_t& from, const NodePtr_t& to) const
      {
        Edges_t edges;
        for (Neighbors_t::const_iterator it = from->neighbors ().begin ();
            it != from->neighbors ().end (); it++) {
          if (it->second->to () == to)
            edges.push_back (it->second);
        }
        return edges;
      }

      EdgePtr_t Graph::chooseEdge (const NodePtr_t& node) const
      {
        return nodeSelector_->chooseEdge (node);
      }

      ConstraintSetPtr_t Graph::configConstraint (const NodePtr_t& node)
      {
        ConstraintSetPtr_t constraint;
        MapFromNode::const_iterator it = constraintSetMapFromNode_.find (node);
        if (it == constraintSetMapFromNode_.end ()) {
          std::string name = toString (node);
          constraint = ConstraintSet::create (robot (), "Set " + name);

          ConfigProjectorPtr_t proj = ConfigProjector::create(robot(), "proj " + name, errorThreshold(), maxIterations());
          insertNumericalConstraints (proj);
          node->insertNumericalConstraints (proj);
          constraint->addConstraint (HPP_DYNAMIC_PTR_CAST(Constraint, proj));

          insertLockedDofs (constraint);
          node->insertLockedDofs (constraint);
          constraintSetMapFromNode_.insert (PairNodeConstraints(node, constraint));
        } else {
          constraint = it->second;
        }
        return constraint;
      }

      ConstraintSetPtr_t Graph::configConstraint (const EdgePtr_t& edge)
      {
        ConstraintSetPtr_t constraint;
        MapFromEdge::const_iterator it = cfgConstraintSetMapFromEdge_.find (edge);
        if (it == cfgConstraintSetMapFromEdge_.end ()) {
          std::string name = toString (edge);
          constraint = ConstraintSet::create (robot (), "Set " + name);

          ConfigProjectorPtr_t proj = ConfigProjector::create(robot(), "proj " + name, errorThreshold(), maxIterations());
          insertNumericalConstraints (proj);
          edge->insertNumericalConstraints (proj);
          edge->to ()->insertNumericalConstraints (proj);
          constraint->addConstraint (HPP_DYNAMIC_PTR_CAST(Constraint, proj));

          insertLockedDofs (constraint);
          edge->insertLockedDofs (constraint);
          edge->to ()->insertLockedDofs (constraint);
          cfgConstraintSetMapFromEdge_.insert (PairEdgeConstraints(edge, constraint));
        } else {
          constraint = it->second;
        }

        return constraint;
      }

      ConstraintSetPtr_t Graph::pathConstraint (const EdgePtr_t& edge)
      {
        ConstraintSetPtr_t constraint;
        MapFromEdge::const_iterator it = pathConstraintSetMapFromEdge_.find (edge);
        if (it == pathConstraintSetMapFromEdge_.end ()) {
          std::string name = toString (edge);
          constraint = ConstraintSet::create (robot (), "Set " + name);

          ConfigProjectorPtr_t proj = ConfigProjector::create(robot(), "proj " + name, errorThreshold(), maxIterations());
          insertNumericalConstraints (proj);
          edge->insertNumericalConstraints (proj);
          edge->node ()->insertNumericalConstraintsForPath (proj);
          constraint->addConstraint (HPP_DYNAMIC_PTR_CAST(Constraint, proj));

          insertLockedDofs (constraint);
          edge->insertLockedDofs (constraint);
          edge->node ()->insertLockedDofs (constraint);
          pathConstraintSetMapFromEdge_.insert (PairEdgeConstraints (edge, constraint));
        } else {
          constraint = it->second;
        }

        return constraint;
      }

      std::ostream& Graph::print (std::ostream& os) const
      {
        return os << (GraphComponent*)this << std::endl << nodeSelector_;
      }
    } // namespace graph
  } // namespace manipulation

} // namespace hpp
