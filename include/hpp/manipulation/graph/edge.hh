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

#ifndef HPP_MANIPULATION_GRAPH_EDGE_HH
# define HPP_MANIPULATION_GRAPH_EDGE_HH

#include <hpp/core/constraint-set.hh>

#include "hpp/manipulation/config.hh"
#include "hpp/manipulation/fwd.hh"
#include "hpp/manipulation/graph/graph.hh"
#include "hpp/manipulation/graph/node.hh"

namespace hpp {
  namespace manipulation {
    namespace graph {
      /// Transition between states of a end-effector.
      ///
      /// Vertices of the graph of constraints.
      class HPP_MANIPULATION_DLLAPI Edge : public GraphComponent
      {
        public:
          /// Create a new empty Edge.
          static EdgePtr_t create (const NodeWkPtr_t& from, const NodeWkPtr_t& to);

          /// Constraint to project onto the same leaf as config.
          /// \return The initialized projector.
          /// \param config Configuration that will initialize the projector.
          ConstraintPtr_t configConstraint(ConfigurationIn_t config);

          /// Constraint to project a path.
          /// \return The initialized constraint.
          /// \param config Configuration that will initialize the constraint.
          ConstraintPtr_t pathConstraint(ConfigurationIn_t config);

          /// Print the object in a stream.
          std::ostream& print (std::ostream& os) const;

          /// Get the destination
          NodePtr_t to () const
          {
            return to_.lock();
          }

          /// Get the origin
          NodePtr_t from () const
          {
            return from_.lock();
          }

        protected:
          /// Initialization of the object.
          void init (const EdgeWkPtr_t& weak, const NodeWkPtr_t& from,
              const NodeWkPtr_t& to);

          /// Constructor
          Edge()
          {}

        private:
          /// The two ends of the transition.
          NodeWkPtr_t from_, to_;

          /// See pathConstraint member function.
          ConstraintPtr_t pathConstraints_;

          /// Constraint ensuring that a q_proj will be in to_ and in the
          /// same leaf of to_ as the configuration used for initialization.
          ConstraintPtr_t configConstraints_;

          /// Weak pointer to itself.
          EdgeWkPtr_t wkPtr_;
      }; // class Edge
    } // namespace graph
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_GRAPH_EDGE_HH
