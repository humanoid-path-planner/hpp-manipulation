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
      Node::Node () : configConstraints_ (new Constraint_t())
      {}

      Node::~Node ()
      {
        if (configConstraints_) delete configConstraints_;
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

      EdgePtr_t Node::linkTo(const NodePtr_t& to, const Weight_t& w, const bool& isInNodeFrom,
          boost::function < EdgePtr_t (const GraphWkPtr_t&, const NodeWkPtr_t&, const NodeWkPtr_t&) > create)
      {
        EdgePtr_t newEdge = create(graph_, wkPtr_, to);
        neighbors_.insert (newEdge, w);
        newEdge->isInNodeFrom (isInNodeFrom);
        return newEdge;
      }

      bool Node::contains (const Configuration_t config)
      {
        return configConstraint()->isSatisfied (config);
      }

      std::ostream& Node::dotPrint (std::ostream& os) const
      {
        os << id () << " [label=\"" << name () << "\"];" << std::endl;
        for (Neighbors_t::const_iterator it = neighbors_.begin();
            it != neighbors_.end(); it++)
          it->second->dotPrint (os) << std::endl;
        return os;
      }

      std::ostream& Node::print (std::ostream& os) const
      {
        os << "|   |-- ";
        GraphComponent::print (os) << std::endl;
        for (Neighbors_t::const_iterator it = neighbors_.begin();
            it != neighbors_.end(); it++)
          os << *(it->second) << " - " << it->first << std::endl;
        return os;
      }

      ConstraintSetPtr_t Node::configConstraint() const
      {
        if (!*configConstraints_) {
          std::string n = "(" + name () + ")";
          GraphPtr_t g = graph_.lock ();
          ConstraintSetPtr_t constraint = ConstraintSet::create (g->robot (), "Set " + n);

          ConfigProjectorPtr_t proj = ConfigProjector::create(g->robot(), "proj " + n, g->errorThreshold(), g->maxIterations());
          // If at least one DifferentiableFunctionPtr_t was inserted, then add the projector.
          if (g->insertNumericalConstraints (proj)
              | insertNumericalConstraints (proj))
            constraint->addConstraint (proj);

          g->insertLockedDofs (constraint);
          insertLockedDofs (constraint);
          configConstraints_->set (constraint);
        }
        return configConstraints_->get ();
      }

      void Node::updateWeight (const EdgePtr_t& e, const Weight_t& w)
      {
        neighbors_.insert (e, w);
      }
    } // namespace graph
  } // namespace manipulation
} // namespace hpp
