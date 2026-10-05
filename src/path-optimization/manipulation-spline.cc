// Copyright (c) 2026 CNRS
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

#include <algorithm>
#include <hpp/core/path-vector.hh>
#include <hpp/core/steering-method/straight.hh>
#include <hpp/manipulation/constraint-set.hh>
#include <hpp/manipulation/graph/edge.hh>
#include <hpp/manipulation/graph/state.hh>
#include <hpp/manipulation/path-optimization/manipulation-spline.hh>

namespace hpp {
namespace manipulation {
namespace pathOptimization {
using core::PathPtr_t;
using core::PathVector;
using core::PathVectorPtr_t;

ManipulationSpline::Ptr_t ManipulationSpline::create(
    const core::ProblemConstPtr_t& problem) {
  ProblemConstPtr_t p(HPP_DYNAMIC_PTR_CAST(const Problem, problem));
  if (!p) throw std::invalid_argument("This is not a manipulation problem.");
  return Ptr_t(new ManipulationSpline(p));
}

ManipulationSpline::ManipulationSpline(const ProblemConstPtr_t& problem)
    : Parent_t(problem) {}

PathVectorPtr_t ManipulationSpline::optimize(const PathVectorPtr_t& path) {
  const size_type nq = path->outputSize(), nv = path->outputDerivativeSize();
  PathVectorPtr_t flat = PathVector::create(nq, nv);
  path->flatten(flat);

  // Group consecutive leaves by transition.
  std::vector<PathVectorPtr_t> transitions;
  std::vector<graph::EdgePtr_t> edges;
  for (std::size_t i = 0; i < flat->numberPaths(); ++i) {
    PathPtr_t leaf = flat->pathAtRank(i);
    if (leaf->length() <= 1e-9) continue;
    ConstraintSetPtr_t c =
        HPP_DYNAMIC_PTR_CAST(ConstraintSet, leaf->constraints());
    if (!c || !c->edge())
      throw std::invalid_argument("Path leaf has no manipulation transition.");
    if (edges.empty() || c->edge() != edges.back()) {
      transitions.push_back(PathVector::create(nq, nv));
      edges.push_back(c->edge());
    }
    transitions.back()->appendPath(leaf);
  }

  // Smooth each transition and join those within the same state.
  std::vector<PathVectorPtr_t> states;
  graph::StatePtr_t previous;
  core::SteeringMethodPtr_t straight =
      core::steeringMethod::Straight::create(problem());
  for (std::size_t i = 0; i < transitions.size(); ++i) {
    PathVectorPtr_t transition = transitions[i];
    if (transition->length() < 1e-6)
      throw std::invalid_argument(
          "Path transition is too short for spline timing");
    if (std::find(singleSplineTransitions.begin(),
                  singleSplineTransitions.end(),
                  edges[i]->name()) != singleSplineTransitions.end()) {
      straight->constraints(transition->pathAtRank(0)->constraints());
      PathPtr_t insertion =
          straight->steer(transition->initial(), transition->end());
      if (!insertion)
        throw std::runtime_error("Failed to fit transition " +
                                 edges[i]->name());
      transition = PathVector::create(nq, nv);
      transition->appendPath(insertion);
    }
    if (edges[i]->state() != previous) {
      states.push_back(PathVector::create(nq, nv));
      previous = edges[i]->state();
    }
    states.back()->concatenate(Parent_t::optimize(transition));
  }

  PathVectorPtr_t result = PathVector::create(nq, nv);
  for (const PathVectorPtr_t& state : states)
    result->appendPath(Parent_t::optimize(state));
  return result;
}

}  // namespace pathOptimization
}  // namespace manipulation
}  // namespace hpp
