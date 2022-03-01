// Copyright (c) 2014, LAAS-CNRS
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.

#ifndef HPP_MANIPULATION_GRAPH_STATE_HH
# define HPP_MANIPULATION_GRAPH_STATE_HH

#include <functional>

#include <hpp/core/constraint-set.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/constraints/implicit.hh>

#include "hpp/manipulation/config.hh"
#include "hpp/manipulation/fwd.hh"
#include "hpp/manipulation/graph/fwd.hh"
#include "hpp/manipulation/graph/edge.hh"
#include "hpp/manipulation/graph/graph-component.hh"

namespace hpp {
  namespace manipulation {
    using constraints::Implicit;
    using constraints::ImplicitPtr_t;
    namespace graph {
      /// \addtogroup constraint_graph
      /// \{

      /// State of an end-effector.
      ///
      /// States of the graph of constraints. There is one
      /// graph for each end-effector.
      class HPP_MANIPULATION_DLLAPI State : public GraphComponent
      {
        public:
	typedef std::function < EdgePtr_t
				(const std::string&,
				 const GraphWkPtr_t&,
				 const StateWkPtr_t&, const StateWkPtr_t&) >
	EdgeFactory;
          /// Destructor
          ~State ();

          /// Create a new state.
          static StatePtr_t create (const std::string& name);

          /// Create a link from this state to the given state.
          /// \param w if strictly negative, the edge is not included in the neighbor
          ///          list. Otherwise, it is included with Weight_t w
          EdgePtr_t linkTo (const std::string& name, const StatePtr_t& to,
			    const size_type& w = 1,
			    EdgeFactory create = Edge::create);

          /// Check whether the configuration is in this state.
          /// \code
          ///   return configConstraint()->isSatisfied (config);
          /// \endcode
          /// \note You should not use this method to know in which states a
          /// configuration is. This only checks if the configuration satisfies
          /// the constraints. Instead, use the class StateSelector.
          virtual bool contains (ConfigurationIn_t config) const;

          inline bool isWaypoint () const
          {
            return isWaypoint_;
          }

          inline void isWaypoint (bool isWaypoint)
          {
            isWaypoint_ = isWaypoint;
          }

          /// Get the parent StateSelector.
          StateSelectorWkPtr_t stateSelector () const
          {
            return selector_;
          }

          /// Set the StateSelector containing this state.
          void stateSelector (const StateSelectorWkPtr_t& parent)
          {
            selector_ = parent;
          };

          /// Get the neighbors
          const Neighbors_t& neighbors() const
          {
            return neighbors_;
          }

          /// Get the neighbors
          Edges_t neighborEdges() const
          {
            return neighbors_.values();
          }

          /// Get the hidden neighbors
          /// It is a vector of transitions outgoing from this state and that are
          /// included in a waypoint edge.
          const Edges_t& hiddenNeighbors() const
          {
            return hiddenNeighbors_;
          }

          /// Set weight of edge starting from this state.
          void updateWeight (const EdgePtr_t&edge, const Weight_t& w);

          /// Get weight of edge starting from this state.
          Weight_t getWeight (const EdgePtr_t&edge);

          /// Constraint to project onto this state.
          ConstraintSetPtr_t configConstraint() const
          {
            throwIfNotInitialized ();
            return configConstraints_;
          }

          /// Add constraint to the state
	  /// Call the parent implementation.
	  /// \throw std::logic_error if the constraint is parameterizable
	  /// (contains at least one Equality comparison type).
          virtual void addNumericalConstraint (
              const ImplicitPtr_t& numConstraint);

          /// Add a constraint for paths that lie in this state.
          virtual void addNumericalConstraintForPath (const ImplicitPtr_t& nm)
          {
            invalidate();
            numericalConstraintsForPath_.push_back (nm);
          }

          /// Insert the numerical constraints in a ConfigProjector
          /// \return true is at least one ImplicitPtr_t was inserted.
          bool insertNumericalConstraintsForPath (ConfigProjectorPtr_t& proj) const
          {
            for (const auto& nc : numericalConstraintsForPath_)
              proj->add (nc);
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
          void init (const StateWkPtr_t& self);

          /// Constructor
          State(const std::string& name);

          /// Print the object in a stream.
          std::ostream& print (std::ostream& os) const;

          virtual void populateTooltip (dot::Tooltip& tp) const;

          virtual void initialize ();

        private:
          /// List of possible motions from this state (i.e. the outgoing
          /// vertices).
          Neighbors_t neighbors_;
          Edges_t hiddenNeighbors_;

          /// Set of constraints to be statisfied.
          ConstraintSetPtr_t configConstraints_;

          /// Stores the numerical constraints for path.
          NumericalConstraints_t numericalConstraintsForPath_;

          /// A selector that will implement the selection of the next state.
          StateSelectorWkPtr_t selector_;

          /// Weak pointer to itself.
          StateWkPtr_t wkPtr_;

          bool isWaypoint_;
      }; // class State

      /// \}
    } // namespace graph
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_GRAPH_STATE_HH
