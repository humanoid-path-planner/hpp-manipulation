//
// Copyright (c) 2016 CNRS
// Authors: Joseph Mirabel
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

#ifndef HPP_MANIPULATION_PROBLEM_TARGET_STATE_HH
#define HPP_MANIPULATION_PROBLEM_TARGET_STATE_HH

#include <hpp/core/fwd.hh>
#include <hpp/core/problem-target.hh>
#include <hpp/manipulation/config.hh>
#include <hpp/manipulation/fwd.hh>
#include <hpp/manipulation/graph/fwd.hh>

namespace hpp {
namespace manipulation {
namespace problemTarget {
/// \addtogroup path_planning
/// \{

/// State
///
/// This class defines a goal using state of the constraint graph.
class HPP_MANIPULATION_DLLAPI State : public core::ProblemTarget {
 public:
  static StatePtr_t create(const core::ProblemPtr_t& problem);

  /// Check if the problem target is well specified.
  void check(const core::RoadmapPtr_t& roadmap) const;

  /// Check whether the problem is solved.
  bool reached(const core::RoadmapPtr_t& roadmap) const;

  core::PathVectorPtr_t computePath(const core::RoadmapPtr_t& roadmap) const;

  void target(const graph::StatePtr_t& state) { state_ = state; }

 protected:
  /// Constructor
  State(const core::ProblemPtr_t& problem) : ProblemTarget(problem) {}

 private:
  graph::StatePtr_t state_;
};  // class State
/// \}
}  // namespace problemTarget
}  //   namespace manipulation
}  // namespace hpp
#endif  // HPP_MANIPULATION_PROBLEM_TARGET_STATE_HH
