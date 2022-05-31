// Copyright (c) 2017, Joseph Mirabel
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

#include <hpp/core/connected-component.hh>
#include <hpp/core/node.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/roadmap.hh>
#include <hpp/manipulation/problem-target/state.hh>
#include <hpp/util/debug.hh>
#include <stdexcept>

#include "astar.hh"

namespace hpp {
namespace manipulation {
namespace problemTarget {
StatePtr_t State::create(const core::ProblemPtr_t& problem) {
  State* tt = new State(problem);
  StatePtr_t shPtr(tt);
  tt->init(shPtr);
  return shPtr;
}

void State::check(const core::RoadmapPtr_t&) const {
  if (!state_) {
    HPP_THROW(std::runtime_error, "No state: task not specified.");
  }
}

bool State::reached(const core::RoadmapPtr_t& roadmap) const {
  const core::ConnectedComponentPtr_t& _cc =
      roadmap->initNode()->connectedComponent();
  const ConnectedComponentPtr_t cc =
      HPP_DYNAMIC_PTR_CAST(ConnectedComponent, _cc);
  assert(cc);
  return !cc->getRoadmapNodes(state_).empty();
}

core::PathVectorPtr_t State::computePath(
    const core::RoadmapPtr_t& roadmap) const {
  core::ProblemPtr_t p = problem_.lock();
  assert(p);
  Astar astar(roadmap, p->distance(), state_);
  return astar.solution();
}
}  // namespace problemTarget
}  // namespace manipulation
}  // namespace hpp
