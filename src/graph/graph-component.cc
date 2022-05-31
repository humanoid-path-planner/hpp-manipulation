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

#include "hpp/manipulation/graph/graph-component.hh"

#include <hpp/constraints/differentiable-function.hh>
#include <hpp/constraints/implicit.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/core/constraint-set.hh>

#include "hpp/manipulation/graph/graph.hh"

namespace hpp {
namespace manipulation {
namespace graph {
typedef constraints::Implicit Implicit;
const std::string& GraphComponent::name() const { return name_; }

std::ostream& GraphComponent::print(std::ostream& os) const {
  os << id() << " : " << name();
  return os;
}

std::ostream& GraphComponent::dotPrint(std::ostream& os,
                                       dot::DrawingAttributes) const {
  os << id();
  return os;
}

void GraphComponent::setDirty() { invalidate(); }

void GraphComponent::addNumericalConstraint(const ImplicitPtr_t& nm) {
  invalidate();
  numericalConstraints_.push_back(nm);
}

void GraphComponent::addNumericalCost(const ImplicitPtr_t& cost) {
  invalidate();
  numericalCosts_.push_back(cost);
}

void GraphComponent::resetNumericalConstraints() {
  invalidate();
  numericalConstraints_.clear();
  numericalCosts_.clear();
}

bool GraphComponent::insertNumericalConstraints(
    ConfigProjectorPtr_t& proj) const {
  for (const auto& nc : numericalConstraints_) proj->add(nc);
  for (const auto& nc : numericalCosts_) proj->add(nc, 1);
  return !numericalConstraints_.empty();
}

const NumericalConstraints_t& GraphComponent::numericalConstraints() const {
  return numericalConstraints_;
}

const NumericalConstraints_t& GraphComponent::numericalCosts() const {
  return numericalCosts_;
}

GraphPtr_t GraphComponent::parentGraph() const { return graph_.lock(); }

void GraphComponent::parentGraph(const GraphWkPtr_t& parent) {
  graph_ = parent;
  GraphPtr_t g = graph_.lock();
  assert(g);
  id_ = g->components().size();
  g->components().push_back(wkPtr_);
}

void GraphComponent::init(const GraphComponentWkPtr_t& weak) { wkPtr_ = weak; }

void GraphComponent::throwIfNotInitialized() const {
  if (!isInit_) {
    throw std::logic_error("The graph should have been initialized first.");
  }
}

std::ostream& operator<<(
    std::ostream& os,
    const hpp::manipulation::graph::GraphComponent& graphComp) {
  return graphComp.print(os);
}

void GraphComponent::populateTooltip(dot::Tooltip& tp) const {
  for (NumericalConstraints_t::const_iterator it =
           numericalConstraints_.begin();
       it != numericalConstraints_.end(); ++it) {
    tp.addLine("- " + (*it)->function().name());
  }
}
}  // namespace graph
}  // namespace manipulation
}  // namespace hpp
