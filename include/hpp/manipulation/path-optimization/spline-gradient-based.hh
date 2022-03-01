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

#ifndef HPP_MANIPULATION_PATH_OPTIMIZATION_SPLINE_GRADIENT_BASED_HH
# define HPP_MANIPULATION_PATH_OPTIMIZATION_SPLINE_GRADIENT_BASED_HH

#include <hpp/core/path-optimization/spline-gradient-based.hh>

#include <hpp/manipulation/config.hh>
#include <hpp/manipulation/problem.hh>

namespace hpp {
  namespace manipulation {
    /// \addtogroup path_optimization
    /// \{
    namespace pathOptimization {
      template <int _PolynomeBasis, int _SplineOrder>
      class HPP_MANIPULATION_DLLAPI SplineGradientBased :
        public core::pathOptimization::SplineGradientBased<_PolynomeBasis, _SplineOrder>
      {
        public:
          enum {
            PolynomeBasis = _PolynomeBasis,
            SplineOrder = _SplineOrder
          };
          typedef core::pathOptimization::SplineGradientBased<PolynomeBasis, SplineOrder>
            Parent_t;
          typedef shared_ptr<SplineGradientBased> Ptr_t;

          using typename Parent_t::Spline;
          using typename Parent_t::SplinePtr_t;
          using typename Parent_t::Splines_t;

          /// Return shared pointer to new object.
          static Ptr_t create (const ProblemConstPtr_t& problem);

          /// This is only for compatibility purpose (with ProblemSolver).
          /// problem is statically casted to an object of type
          /// const manipulation::Problem& and method create(const Problem&)
          /// is called.
          static Ptr_t createFromCore (const core::ProblemConstPtr_t& problem);

        protected:
          typedef typename hpp::core::pathOptimization::LinearConstraint LinearConstraint;
          using typename Parent_t::SplineOptimizationDatas_t;

          SplineGradientBased(const ProblemConstPtr_t& problem);

          /// Get path validation for each spline
          ///
          /// \param splines, vector of splines
          ///
          /// for each spline in the input vector, retrieve the path validation
          /// method of the transition the spline comes from.
          /// If no edge is found, use path validation in problem.
          ///
          /// \note path validation methods are stored in member
          /// hpp::core::pathOptimization::SplineGradientBasedAbstract::validations_
          virtual void initializePathValidation(const Splines_t& splines);

          virtual void addProblemConstraints (const core::PathVectorPtr_t& init,
              const Splines_t& splines, LinearConstraint& lc, SplineOptimizationDatas_t& sods) const;

          virtual void constrainEndIntoState (const core::PathPtr_t& path,
              const size_type& idxSpline, const SplinePtr_t& spline,
              const graph::StatePtr_t state, LinearConstraint& lc) const;

          virtual void constraintDerivativesAtEndOfSpline (const size_type& idxSpline,
              const SplinePtr_t& spline, LinearConstraint& lc) const;
      }; // SplineGradientBased
    } // namespace pathOptimization
    /// \}
  }  // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_PATH_OPTIMIZATION_GRADIENT_BASED_HH
