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

#include <hpp/manipulation/path-optimization/spline-gradient-based.hh>

#include <hpp/core/path-optimization/spline-gradient-based/linear-constraint.hh>

#include <hpp/manipulation/constraint-set.hh>
#include <hpp/manipulation/graph/state.hh>
#include <hpp/manipulation/graph/edge.hh>

namespace hpp {
  namespace manipulation {
    namespace pathOptimization {

      template <int _PB, int _SO>
      SplineGradientBased<_PB, _SO>::SplineGradientBased (const Problem& problem)
        : Parent_t (problem)
      {
        this->checkOptimum_ = true;
      }

      // ----------- Convenience class -------------------------------------- //

      // ----------- Resolution steps --------------------------------------- //

      template <int _PB, int _SO>
      typename SplineGradientBased<_PB, _SO>::Ptr_t SplineGradientBased<_PB, _SO>::create
      (const Problem& problem)
      {
	SplineGradientBased* ptr = new SplineGradientBased (problem);
	Ptr_t shPtr (ptr);
	return shPtr;
      }

      template <int _PB, int _SO>
      typename SplineGradientBased<_PB, _SO>::Ptr_t SplineGradientBased<_PB, _SO>::createFromCore
      (const core::Problem& problem)
      {
        HPP_STATIC_CAST_REF_CHECK(const Problem, problem);
        return create (static_cast<const Problem&>(problem));
      }

      template <int _PB, int _SO>
      void SplineGradientBased<_PB, _SO>::initializePathValidation
      (const Splines_t& splines)
      {
        this->validations_.resize(splines.size());
        for (std::size_t i = 0; i < splines.size(); ++i) {
          ConstraintSetPtr_t set = HPP_STATIC_PTR_CAST (ConstraintSet, splines[i]->constraints ());
          if (set && set->edge())
            this->validations_[i] = set->edge()->pathValidation();
          else
            this->validations_[i] = this->problem().pathValidation();
        }
      }

      template <int _PB, int _SO>
      void SplineGradientBased<_PB, _SO>::addProblemConstraints
      (const core::PathVectorPtr_t& init, const Splines_t& splines, LinearConstraint& lc, Solvers_t& ss) const
      {
        assert (init->numberPaths() == splines.size() && ss.size() == splines.size());

        const std::size_t last = splines.size() - 1;
        bool prevReversed;
        for (std::size_t i = 0; i < last; ++i) {
          core::PathPtr_t path = init->pathAtRank(i);
          ConstraintSetPtr_t set = HPP_STATIC_PTR_CAST (ConstraintSet, splines[i]->constraints ());
          if (!set || !set->edge())
            std::invalid_argument ("Cannot optimize a path that has not been "
                "generated with a graph.");
          graph::EdgePtr_t transition = set->edge();

          this->addProblemConstraintOnPath (path, i, splines[i], lc, ss[i]);

          bool reversed = transition->direction (path);

          // The path should always go through the start and end states of the
          // transition.
          // FIXME problem of waypoint edges...
          graph::WaypointEdgePtr_t we = HPP_DYNAMIC_PTR_CAST(graph::WaypointEdge, transition);
          if (we) reversed = prevReversed;
          graph::StatePtr_t from = (we ? we->waypoint<graph::Edge>(we->nbWaypoints() - 1)->to() : transition->from());
          if (reversed && transition->state() != from) {
            // Do something different
            constrainEndIntoState (path, i, splines[i], from, lc);
          } else if (!reversed && transition->state() != transition->to()) {
            // Do something different
            constrainEndIntoState (path, i, splines[i], transition->to(), lc);
          }
          prevReversed = reversed;
        }
        this->addProblemConstraintOnPath (init->pathAtRank(last), last, splines[last], lc, ss[last]);
      }

      template <int _PB, int _SO>
      void SplineGradientBased<_PB, _SO>::constrainEndIntoState
      (const core::PathPtr_t& path, const size_type& idxSpline,
       const SplinePtr_t& spline, const graph::StatePtr_t state,
       LinearConstraint& lc) const
      {
        typename Spline::BasisFunctionVector_t B1;
        spline->basisFunctionDerivative(0, 1, B1);
        // TODO Should we add zero velocity sometimes ?

        ConstraintSetPtr_t set = state->configConstraint();
        value_type guessThreshold = this->problem().getParameter ("SplineGradientBased/guessThreshold", value_type(-1));
        Eigen::RowBlockIndexes select =
          this->computeActiveParameters (path, set->configProjector()->solver(), guessThreshold);

        const size_type rDof = this->robot_->numberDof(),
                        col  = idxSpline * Spline::NbCoeffs * rDof,
                        row = lc.J.rows(),
                        nOutVar = select.nbIndexes();

        // Add nOutVar constraints
        lc.addRows(nOutVar);
        matrix_t I = select.rview(matrix_t::Identity(rDof, rDof));
        for (size_type k = 0; k < Spline::NbCoeffs; ++k)
          lc.J.block  (row, col + k * rDof, nOutVar, rDof) = B1(k) * I;
        lc.b.segment(row, nOutVar) = select.rview(spline->parameters().transpose() * B1);

        assert ((lc.J.block(row, col, nOutVar, rDof * Spline::NbCoeffs) * spline->rowParameters())
            .isApprox(lc.b.segment(row, nOutVar)));
      }

      // ----------- Optimize ----------------------------------------------- //

      // ----------- Convenience functions ---------------------------------- //

      // ----------- Instanciate -------------------------------------------- //

      template class SplineGradientBased<core::path::CanonicalPolynomeBasis, 1>; // equivalent to StraightPath
      template class SplineGradientBased<core::path::CanonicalPolynomeBasis, 2>;
      template class SplineGradientBased<core::path::CanonicalPolynomeBasis, 3>;
      template class SplineGradientBased<core::path::BernsteinBasis, 1>; // equivalent to StraightPath
      template class SplineGradientBased<core::path::BernsteinBasis, 2>;
      template class SplineGradientBased<core::path::BernsteinBasis, 3>;
    } // namespace pathOptimization
  }  // namespace manipulation
} // namespace hpp
