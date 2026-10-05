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

#ifndef HPP_MANIPULATION_PATH_OPTIMIZATION_MANIPULATION_SPLINE_HH
#define HPP_MANIPULATION_PATH_OPTIMIZATION_MANIPULATION_SPLINE_HH

#include <hpp/manipulation/path-optimization/spline-gradient-based.hh>

namespace hpp {
namespace manipulation {
namespace pathOptimization {
/// \addtogroup path_optimization
/// \{

/// Smooth transitions, then join splines within each manipulation state.
///
/// The result contains one PathVector per consecutive group of transitions
/// with the same containing state. These groups can be timed independently
/// to stop at state changes.
///
/// \pre Input paths are geometric and each leaf carries a
/// hpp::manipulation::ConstraintSet with the correct transition.
class HPP_MANIPULATION_DLLAPI ManipulationSpline
    : public SplineGradientBased<core::path::BernsteinBasis, 3> {
 public:
  typedef SplineGradientBased<core::path::BernsteinBasis, 3> Parent_t;
  typedef shared_ptr<ManipulationSpline> Ptr_t;

  static Ptr_t create(const core::ProblemConstPtr_t& problem);

  virtual core::PathVectorPtr_t optimize(const core::PathVectorPtr_t& path);

  /// Transitions fitted with one spline between their endpoints, using their
  /// constraints, for example constrained insertion motions. Other transitions
  /// keep their interpolation intervals as initial spline pieces.
  std::vector<std::string> singleSplineTransitions;

 protected:
  ManipulationSpline(const ProblemConstPtr_t& problem);
};  // ManipulationSpline

/// \}
}  // namespace pathOptimization
}  // namespace manipulation
}  // namespace hpp

#endif  // HPP_MANIPULATION_PATH_OPTIMIZATION_MANIPULATION_SPLINE_HH
