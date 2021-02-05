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

#ifndef HPP_MANIPULATION_GRAPH_GRAPHCOMPONENT_HH
# define HPP_MANIPULATION_GRAPH_GRAPHCOMPONENT_HH

# include <string>
# include <ostream>

# include "hpp/manipulation/config.hh"
# include "hpp/manipulation/deprecated.hh"
# include "hpp/manipulation/fwd.hh"
# include "hpp/manipulation/graph/fwd.hh"
# include "hpp/manipulation/graph/dot.hh"

namespace hpp {
  namespace manipulation {
    typedef constraints::ImplicitPtr_t ImplicitPtr_t;
    namespace graph {
      /// \defgroup constraint_graph Constraint Graph

      /// \addtogroup constraint_graph
      /// \{

      /// Define common methods of the graph components.
      class HPP_MANIPULATION_DLLAPI GraphComponent
      {
        public:
          virtual ~GraphComponent () {};

          /// Get the component name.
          const std::string& name() const;

          /// Return the component id.
	  const std::size_t& id () const
          {
            return id_;
          }

          /// Add constraint to the component.
          virtual void addNumericalConstraint (
              const ImplicitPtr_t& numConstraint);

          /// Add a cost function Implicit to the component.
          virtual void addNumericalCost (const ImplicitPtr_t& numCost);

	  /// Reset the numerical constraints stored in the component.
	  virtual void resetNumericalConstraints ();

          /// Insert the numerical constraints in a ConfigProjector
          /// \return true is at least one ImplicitPtr_t was inserted.
          bool insertNumericalConstraints (ConfigProjectorPtr_t& proj) const;

          /// Get a vector of numerical constraints of the instance
          /// \return reference to member numericalConstraints_ except for
          ///         class Edge that may merge constraints with their
          ///         complement.
          virtual const NumericalConstraints_t& numericalConstraints() const;

          /// Get a reference to the NumericalConstraints_t
          const NumericalConstraints_t& numericalCosts() const;

          /// Set the parent graph.
          void parentGraph(const GraphWkPtr_t& parent);

          /// Set the parent graph.
          GraphPtr_t parentGraph() const;

          /// Print the component in DOT language.
          virtual std::ostream& dotPrint (std::ostream& os, dot::DrawingAttributes da = dot::DrawingAttributes ()) const;

          /// Invalidate the component
          /// The component needs to be initialized again.
          virtual void invalidate()
          {
            isInit_ = false;
          }

          /// Declare a component as dirty
          /// \deprecated call invalidate instead
          void setDirty() HPP_MANIPULATION_DEPRECATED;

        protected:
          /// Initialize the component
          void init (const GraphComponentWkPtr_t& weak);

          GraphComponent(const std::string& name) : isInit_(false)
                                                    , name_ (name)
                                                    , id_(-1)
          {}

          /// Stores the numerical constraints.
          NumericalConstraints_t numericalConstraints_;
          /// Stores the numerical costs.
          NumericalConstraints_t numericalCosts_;
          /// A weak pointer to the parent graph.
          GraphWkPtr_t graph_;

          bool isInit_;

          void throwIfNotInitialized () const;

          /// Print the object in a stream.
          virtual std::ostream& print (std::ostream& os) const;
          friend std::ostream& operator<< (std::ostream&, const GraphComponent&);

          /// Populate DrawingAttributes tooltip
          virtual void populateTooltip (dot::Tooltip& tp) const;

          virtual void initialize() = 0;

        private:
          /// Name of the component.
          std::string name_;
          /// Weak pointer to itself.
          GraphComponentWkPtr_t wkPtr_;
          /// ID of the component (index in components vector).
	  std::size_t id_;

          friend class Graph;
      };

      std::ostream& operator<< (std::ostream& os, const GraphComponent& graphComp);

      /// \}
    } // namespace graph
  } // namespace manipulation

} // namespace hpp

#endif // HPP_MANIPULATION_GRAPH_GRAPHCOMPONENT_HH
