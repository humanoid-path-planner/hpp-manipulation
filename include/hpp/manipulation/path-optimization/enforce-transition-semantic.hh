// Copyright (c) 2018, Joseph Mirabel
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

#ifndef HPP_MANIPULATION_PATHOPTIMIZATION_ENFORCE_TRANSITION_SEMANTIC_HH
#define HPP_MANIPULATION_PATHOPTIMIZATION_ENFORCE_TRANSITION_SEMANTIC_HH

#include <hpp/core/path-optimizer.hh>
#include <hpp/manipulation/problem.hh>

namespace hpp {
namespace manipulation {
namespace pathOptimization {
using hpp::core::Path;
using hpp::core::PathPtr_t;
using hpp::core::PathVector;
/// \addtogroup path_optimization
/// \{

/// Recompute the transition relative to each element of the path vector
///
/// When executing a sequence of direct paths on a real robot, it is useful to know which
/// transition of the graph each direct path corresponds to. For example, in a manipulation
/// motion, before grasping an object, the robot needs to open the gripper. This information
/// is contained in the transition that leads to a pregrasp waypoint state. The direct path
/// should therefore have access to the transition.
///
/// The information is stored in the \link hpp::manipulation::ConstraintSet constraint set \endlink
/// of the path and is accessible via method \link hpp::manipulation::ConstraintSet::edge edge
/// \endlink.
///
/// If the path vector is produced by a manipulation planner, each direct path has been created
/// by a transition. However, the path may later be cut by random shortcut or due to collision and
/// the associated transition become irrelevant. For example if a path is created by a transition
/// that leads to a pre-grasp, and cut due to a collision, the path does not reach the target
/// state and the relevant transition is not the one that built the path.
///
/// This class takes a \link hpp::core::PathVector path vector \endlink as input an relabel
/// each direct path with the correct transition.
///
/// \pre The path should have been created by a manipulation planning algorithm: in other
/// words, the constraint set of each direct path should be of type
/// hpp::manipulation::ConstraintSet.
class HPP_MANIPULATION_DLLAPI EnforceTransitionSemantic
    : public core::PathOptimizer {
 public:
  typedef hpp::core::PathVectorPtr_t PathVectorPtr_t;
  typedef shared_ptr<EnforceTransitionSemantic> Ptr_t;

  static Ptr_t create(const core::ProblemConstPtr_t& problem) {
    ProblemConstPtr_t p(HPP_DYNAMIC_PTR_CAST(const Problem, problem));
    if (!p) throw std::invalid_argument("This is not a manipulation problem.");
    return Ptr_t(new EnforceTransitionSemantic(p));
  }

  virtual PathVectorPtr_t optimize(const PathVectorPtr_t& path);

 protected:
  /// Constructor
  EnforceTransitionSemantic(const ProblemConstPtr_t& problem)
      : PathOptimizer(problem), problem_(problem) {}

 private:
  ProblemConstPtr_t problem_;
};

/// \}
}  // namespace pathOptimization
}  // namespace manipulation
}  // namespace hpp

#endif  // HPP_MANIPULATION_PATHOPTIMIZATION_ENFORCE_TRANSITION_SEMANTIC_HH
