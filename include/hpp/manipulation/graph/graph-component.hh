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

#ifndef HPP_MANIPULATION_GRAPH_GRAPHCOMPONENT_HH
#define HPP_MANIPULATION_GRAPH_GRAPHCOMPONENT_HH

#include <ostream>
#include <string>

#include "hpp/manipulation/config.hh"
#include "hpp/manipulation/deprecated.hh"
#include "hpp/manipulation/fwd.hh"
#include "hpp/manipulation/graph/dot.hh"
#include "hpp/manipulation/graph/fwd.hh"

namespace hpp {
namespace manipulation {
typedef constraints::ImplicitPtr_t ImplicitPtr_t;
namespace graph {
/// \defgroup constraint_graph Constraint Graph

/// \addtogroup constraint_graph
/// \{

/// Define common methods of the graph components.
class HPP_MANIPULATION_DLLAPI GraphComponent {
 public:
  virtual ~GraphComponent(){};

  /// Get the component name.
  const std::string& name() const;

  /// Return the component id.
  const std::size_t& id() const { return id_; }

  /// Add constraint to the component.
  virtual void addNumericalConstraint(const ImplicitPtr_t& numConstraint);

  /// Add a cost function Implicit to the component.
  virtual void addNumericalCost(const ImplicitPtr_t& numCost);

  /// Reset the numerical constraints stored in the component.
  virtual void resetNumericalConstraints();

  /// Insert the numerical constraints in a ConfigProjector
  /// \return true is at least one ImplicitPtr_t was inserted.
  bool insertNumericalConstraints(ConfigProjectorPtr_t& proj) const;

  /// Get a reference to the NumericalConstraints_t
  const NumericalConstraints_t& numericalConstraints() const;

  /// Get a reference to the NumericalConstraints_t
  const NumericalConstraints_t& numericalCosts() const;

  /// Set the parent graph.
  void parentGraph(const GraphWkPtr_t& parent);

  /// Set the parent graph.
  GraphPtr_t parentGraph() const;

  /// Print the component in DOT language.
  virtual std::ostream& dotPrint(
      std::ostream& os,
      dot::DrawingAttributes da = dot::DrawingAttributes()) const;

  /// Invalidate the component
  /// The component needs to be initialized again.
  virtual void invalidate() { isInit_ = false; }

  /// Declare a component as dirty
  /// \deprecated call invalidate instead
  HPP_MANIPULATION_DEPRECATED void setDirty();

  /// Set whether hierachical constraints are solved level by level
  /// \sa hpp::constraints::solver::HierarchicalIterative
  void solveLevelByLevel(bool solveLevelByLevel) {
    solveLevelByLevel_ = solveLevelByLevel;
  }

  /// Get whether hierachical constraints are solved level by level
  /// \sa hpp::constraints::solver::HierarchicalIterative
  bool solveLevelByLevel() const { return solveLevelByLevel_; }

 protected:
  /// Initialize the component
  void init(const GraphComponentWkPtr_t& weak);

  GraphComponent(const std::string& name)
      : isInit_(false), name_(name), id_(-1), solveLevelByLevel_(false) {}

  /// Stores the numerical constraints.
  NumericalConstraints_t numericalConstraints_;
  /// Stores the numerical costs.
  NumericalConstraints_t numericalCosts_;
  /// A weak pointer to the parent graph.
  GraphWkPtr_t graph_;

  bool isInit_;

  void throwIfNotInitialized() const;

  /// Print the object in a stream.
  virtual std::ostream& print(std::ostream& os) const;
  friend std::ostream& operator<<(std::ostream&, const GraphComponent&);

  /// Populate DrawingAttributes tooltip
  virtual void populateTooltip(dot::Tooltip& tp) const;

  virtual void initialize() = 0;

 private:
  /// Name of the component.
  std::string name_;
  /// Weak pointer to itself.
  GraphComponentWkPtr_t wkPtr_;
  /// ID of the component (index in components vector).
  std::size_t id_;
  /// Whether the constraints are solved level by level
  bool solveLevelByLevel_;
  friend class Graph;
};

std::ostream& operator<<(std::ostream& os, const GraphComponent& graphComp);

/// \}
}  // namespace graph
}  // namespace manipulation

}  // namespace hpp

#endif  // HPP_MANIPULATION_GRAPH_GRAPHCOMPONENT_HH
