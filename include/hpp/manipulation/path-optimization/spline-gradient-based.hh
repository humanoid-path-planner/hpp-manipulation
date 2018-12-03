// Copyright (c) 2017, Joseph Mirabel
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
          typedef boost::shared_ptr<SplineGradientBased> Ptr_t;

          using typename Parent_t::Spline;
          using typename Parent_t::SplinePtr_t;
          using typename Parent_t::Splines_t;

          /// Return shared pointer to new object.
          static Ptr_t create (const Problem& problem);

          /// This is only for compatibility purpose (with ProblemSolver).
          /// problem is statically casted to an object of type
          /// const manipulation::Problem& and method create(const Problem&)
          /// is called.
          static Ptr_t createFromCore (const core::Problem& problem);

        protected:
          typedef typename hpp::core::pathOptimization::LinearConstraint LinearConstraint;
          using typename Parent_t::SplineOptimizationDatas_t;

          SplineGradientBased (const Problem& problem);

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
