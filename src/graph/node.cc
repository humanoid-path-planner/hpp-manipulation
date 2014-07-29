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

#include "hpp/manipulation/robot.hh"
#include "hpp/manipulation/graph/edge.hh"
#include "hpp/manipulation/graph/graph.hh"
#include "hpp/manipulation/graph/node.hh"

namespace hpp {
  namespace manipulation {
    namespace graph {
      ConfigProjectorPtr_t buildConfigProjector (GraphWkPtr_t graph, const std::string& name)
      {
        GraphPtr_t g = graph.lock ();
        if (!g) {
          HPP_THROW_EXCEPTION(Bad_function_call, "Invalid weak_ptr to the Graph.");
        }

        ConfigProjectorPtr_t ret = ConfigProjector::
          create(g->robot(), name, g->errorThreshold(), g->maxIterations());
        return ret;
      }

      ConstraintSetPtr_t buildConstraintSet (GraphWkPtr_t graph, const std::string& name)
      {
        GraphPtr_t g = graph.lock ();
        if (!g) {
          HPP_THROW_EXCEPTION(Bad_function_call, "Invalid weak_ptr to the Graph.");
        }

        ConstraintSetPtr_t ret = ConstraintSet::create(g->robot(), name);
        return ret;
      }

      NodePtr_t Node::create ()
      {
        Node* node = new Node;
        NodePtr_t shPtr(node);
        shPtr->init(shPtr);
        return shPtr;
      }

      void Node::init (const NodeWkPtr_t& weak)
      {
        GraphComponent::init (weak);
        wkPtr_ = weak;
      }

      EdgePtr_t Node::linkTo(const NodePtr_t& to)
      {
        EdgePtr_t newEdge = Edge::create(wkPtr_, to);
        neighbors_.push_back(newEdge);
        newEdge->parentGraph(graph_);
        return newEdge;
      }

      bool Node::contains (const Configuration_t config)
      {
        // TODO: This is not the most efficient way. We should
        // compute the value of the constraint instead of apllying
        // the constraint.
        Configuration_t cfg = config;
        return configConstraint()->apply(cfg) && ( cfg == config );
      }

      std::ostream& Node::print (std::ostream& os) const
      {
        GraphComponent::print (os << "|   |-- ") << std::endl;
        for (Edges_t::const_iterator it = neighbors_.begin();
            it != neighbors_.end(); it++)
          os << *(*it);
        return os;
      }

      ConstraintPtr_t Node::configConstraint()
      {
        if (!configConstraints_) {
          ConstraintSetPtr_t configConst = buildConstraintSet (graph_, name () + "-cfgconstraint");
          insertListIn <LockedDofs_t> (lockedDofConstraints_, configConst);
          if (numericalConstraints_.size () > 0) {
            ConfigProjectorPtr_t cp = buildConfigProjector (graph_, name () + "cfgproj");
            insertListIn <DifferentiableFunctions_t> (numericalConstraints_, cp);
            configConst->addConstraint (HPP_DYNAMIC_PTR_CAST(Constraint, cp));
          }
          configConstraints_ = configConst;
        }
        return configConstraints_;
      }
    } // namespace graph
  } // namespace manipulation
} // namespace hpp
