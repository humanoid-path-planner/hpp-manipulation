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

#include "hpp/manipulation/steering-method/graph.hh"

#include <hpp/core/steering-method/straight.hh>
#include <hpp/core/straight-path.hh>
#include <hpp/manipulation/graph/edge.hh>
#include <hpp/manipulation/graph/graph.hh>
#include <hpp/manipulation/problem.hh>
#include <hpp/util/pointer.hh>

namespace hpp {
namespace manipulation {
SteeringMethod::SteeringMethod(const ProblemConstPtr_t& problem)
    : core::SteeringMethod(problem),
      problem_(problem),
      steeringMethod_(core::steeringMethod::Straight::create(problem)) {}

SteeringMethod::SteeringMethod(const SteeringMethod& other)
    : core::SteeringMethod(other),
      problem_(other.problem_),
      steeringMethod_(other.steeringMethod_) {}

namespace steeringMethod {

GraphPtr_t Graph::create(const core::ProblemConstPtr_t& problem) {
  assert(HPP_DYNAMIC_PTR_CAST(const Problem, problem));
  ProblemConstPtr_t p = HPP_STATIC_PTR_CAST(const Problem, problem);
  Graph* ptr = new Graph(p);
  GraphPtr_t shPtr(ptr);
  ptr->init(shPtr);
  return shPtr;
}

GraphPtr_t Graph::createCopy(const GraphPtr_t& other) {
  Graph* ptr = new Graph(*other);
  GraphPtr_t shPtr(ptr);
  ptr->init(shPtr);
  return shPtr;
}

Graph::Graph(const ProblemConstPtr_t& problem)
    : SteeringMethod(problem), weak_() {}

Graph::Graph(const Graph& other) : SteeringMethod(other) {}

PathPtr_t Graph::impl_compute(ConfigurationIn_t q1,
                              ConfigurationIn_t q2) const {
  graph::Edges_t possibleEdges;
  // If q1 and q2 are the same, call the problem steering method between
  // them
  if (q1 == q2) {
    core::SteeringMethodPtr_t sm(
        problem_->manipulationSteeringMethod()->innerSteeringMethod());
    return (*sm)(q1, q2);
  }
  if (!problem_->constraintGraph())
    throw std::invalid_argument(
        "The constraint graph should be set to use the steeringMethod::Graph");
  const graph::Graph& graph = *(problem_->constraintGraph());
  try {
    possibleEdges = graph.getEdges(graph.getState(q1), graph.getState(q2));
  } catch (const std::logic_error& e) {
    hppDout(error, e.what());
    return PathPtr_t();
  }
  PathPtr_t path;
  if (possibleEdges.empty()) {
    hppDout(info, "No edge found.");
  }
  while (!possibleEdges.empty()) {
    if (possibleEdges.back()->build(path, q1, q2)) {
      return path;
    }
    possibleEdges.pop_back();
  }
  return PathPtr_t();
}
}  // namespace steeringMethod
}  // namespace manipulation
}  // namespace hpp
