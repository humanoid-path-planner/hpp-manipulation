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

#ifndef HPP_MANIPULATION_GRAPH_NODE_HH
# define HPP_MANIPULATION_GRAPH_NODE_HH

# include <boost/function.hpp>

#include <hpp/core/locked-joint.hh>
#include <hpp/core/constraint-set.hh>
#include <hpp/core/config-projector.hh>

#include "hpp/manipulation/config.hh"
#include "hpp/manipulation/deprecated.hh"
#include "hpp/manipulation/fwd.hh"
#include "hpp/manipulation/graph/fwd.hh"
#include "hpp/manipulation/graph/edge.hh"
#include "hpp/manipulation/graph/graph-component.hh"

namespace hpp {
  namespace manipulation {
    namespace graph {
      /// \addtogroup constraint_graph
      /// \{

      /// State of an end-effector.
      ///
      /// Nodes of the graph of constraints. There is one
      /// graph for each end-effector.
      class HPP_MANIPULATION_DLLAPI Node : public GraphComponent
      {
        public:
	typedef boost::function < EdgePtr_t
				  (const std::string&,
				   const GraphWkPtr_t&,
				   const NodeWkPtr_t&, const NodeWkPtr_t&) >
	EdgeFactory;
          /// Destructor
          ~Node ();

          /// Create a new node.
          static NodePtr_t create (const std::string& name);

          /// Create a link from this node to the given node.
          /// \param w if strictly negative, the edge is not included in the neighbor
          ///          list. Otherwise, it is included with Weight_t w
          EdgePtr_t linkTo (const std::string& name, const NodePtr_t& to,
			    const size_type& w = 1,
			    EdgeFactory create = Edge::create);

          /// Check whether the configuration is in this state.
          /// \code
          ///   return configConstraint()->isSatisfied (config);
          /// \endcode
          /// \note You should not use this method to know in which states a
          /// configuration is. This only checks if the configuration satisfies
          /// the constraints. Instead, use the class NodeSelector.
          virtual bool contains (ConfigurationIn_t config) const;

          inline bool isWaypoint () const
          {
            return isWaypoint_;
          }

          inline void isWaypoint (bool isWaypoint)
          {
            isWaypoint_ = isWaypoint;
          }

          /// Get the parent NodeSelector.
          NodeSelectorWkPtr_t nodeSelector () const
          {
            return selector_;
          }

          /// Set the NodeSelector containing this node.
          void nodeSelector (const NodeSelectorWkPtr_t& parent)
          {
            selector_ = parent;
          };

          /// Get the neighbors
          const Neighbors_t& neighbors() const
          {
            return neighbors_;
          }

          void updateWeight (const EdgePtr_t&edge, const Weight_t& w);

          /// Constraint to project onto this node.
          ConstraintSetPtr_t configConstraint() const;

          /// Add core::NumericalConstraint to the component.
          virtual void addNumericalConstraintForPath (const NumericalConstraintPtr_t& nm,
              const SizeIntervals_t& passiveDofs = SizeIntervals_t ())
          {
            numericalConstraintsForPath_.push_back (nm);
            passiveDofsForPath_.push_back (passiveDofs);
          }

          /// Add core::DifferentiableFunction to the component.
          virtual void addNumericalConstraintForPath (const DifferentiableFunctionPtr_t& function, const ComparisonTypePtr_t& ineq)
            HPP_MANIPULATION_DEPRECATED
          {
            numericalConstraintsForPath_.push_back (NumericalConstraint::create (function,ineq));
          }

          /// Insert the numerical constraints in a ConfigProjector
          /// \return true is at least one NumericalConstraintPtr_t was inserted.
          bool insertNumericalConstraintsForPath (ConfigProjectorPtr_t& proj) const
          {
            assert (numericalConstraintsForPath_.size () == passiveDofsForPath_.size ());
            IntervalsContainer_t::const_iterator itpdofs = passiveDofsForPath_.begin ();
            for (NumericalConstraints_t::const_iterator it = numericalConstraintsForPath_.begin();
                it != numericalConstraintsForPath_.end(); it++) {
              proj->add (*it, *itpdofs);
              itpdofs++;
            }
            return !numericalConstraintsForPath_.empty ();
          }

          /// Get a reference to the NumericalConstraints_t
          const NumericalConstraints_t& numericalConstraintsForPath () const
          {
            return numericalConstraintsForPath_;
          }

          /// Print the object in a stream.
          std::ostream& dotPrint (std::ostream& os, dot::DrawingAttributes da = dot::DrawingAttributes ()) const;

        protected:
          /// Initialize the object.
          void init (const NodeWkPtr_t& self);

          /// Constructor
          Node(const std::string& name);

          /// Print the object in a stream.
          std::ostream& print (std::ostream& os) const;

          virtual void populateTooltip (dot::Tooltip& tp) const;

        private:
          /// List of possible motions from this state (i.e. the outgoing
          /// vertices).
          Neighbors_t neighbors_;
          std::vector <EdgePtr_t> hiddenNeighbors_;

          /// Set of constraints to be statisfied.
          typedef Cache < ConstraintSetPtr_t > Constraint_t;
          Constraint_t* configConstraints_;

          /// Stores the numerical constraints for path.
          NumericalConstraints_t numericalConstraintsForPath_;
          IntervalsContainer_t passiveDofsForPath_;

          /// A selector that will implement the selection of the next state.
          NodeSelectorWkPtr_t selector_;

          /// Weak pointer to itself.
          NodeWkPtr_t wkPtr_;

          bool isWaypoint_;
      }; // class Node

      /// \}
    } // namespace graph
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_GRAPH_NODE_HH
