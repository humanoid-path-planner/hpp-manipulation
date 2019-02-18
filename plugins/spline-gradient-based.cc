// Copyright (c) 2019, Joseph Mirabel
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

#include <hpp/core/plugin.hh>
#include <hpp/core/problem-solver.hh>

#include <hpp/manipulation/path-optimization/spline-gradient-based.hh>

namespace hpp {
  namespace manipulation {
    class SplineGradientBasedPlugin : public core::ProblemSolverPlugin
    {
      public:
        SplineGradientBasedPlugin ()
          : ProblemSolverPlugin ("SplineGradientBasedPlugin", "0.0")
        {}

      protected:
        virtual bool impl_initialize (core::ProblemSolverPtr_t ps)
        {
          // ps->pathOptimizers.add ("SplineGradientBased_cannonical1",pathOptimization::SplineGradientBased<core::path::CanonicalPolynomeBasis, 1>::createFromCore);
          // ps->pathOptimizers.add ("SplineGradientBased_cannonical2",pathOptimization::SplineGradientBased<core::path::CanonicalPolynomeBasis, 2>::createFromCore);
          // ps->pathOptimizers.add ("SplineGradientBased_cannonical3",pathOptimization::SplineGradientBased<core::path::CanonicalPolynomeBasis, 3>::createFromCore);
          ps->pathOptimizers.add ("SplineGradientBased_bezier1",pathOptimization::SplineGradientBased<core::path::BernsteinBasis, 1>::createFromCore);
          // ps->pathOptimizers.add ("SplineGradientBased_bezier2",pathOptimization::SplineGradientBased<core::path::BernsteinBasis, 2>::createFromCore);
          ps->pathOptimizers.add ("SplineGradientBased_bezier3",pathOptimization::SplineGradientBased<core::path::BernsteinBasis, 3>::createFromCore);

          return true;
        }
    };
  } // namespace manipulation
} // namespace hpp

HPP_CORE_DEFINE_PLUGIN(hpp::manipulation::SplineGradientBasedPlugin)
