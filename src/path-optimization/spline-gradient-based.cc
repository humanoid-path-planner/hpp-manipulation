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

        bool zeroDerivative = this->problem().getParameter ("SplineGradientBased/zeroDerivativesAtStateIntersection", false);

        const std::size_t last = splines.size() - 1;
        graph::StatePtr_t stateOfStart;
        for (std::size_t i = 0; i < last; ++i) {
          core::PathPtr_t path = init->pathAtRank(i);
          ConstraintSetPtr_t set = HPP_STATIC_PTR_CAST (ConstraintSet, splines[i]->constraints ());
          if (!set || !set->edge())
            std::invalid_argument ("Cannot optimize a path that has not been "
                "generated with a graph.");
          graph::EdgePtr_t transition = set->edge();

          this->addProblemConstraintOnPath (path, i, splines[i], lc, ss[i]);

          // The path should always go through the start and end states of the
          // transition.
          // FIXME problem of waypoint edges...
          graph::WaypointEdgePtr_t we = HPP_DYNAMIC_PTR_CAST(graph::WaypointEdge, transition);
          graph::StatePtr_t from = (we ? we->waypoint<graph::Edge>(we->nbWaypoints() - 1)->to() : transition->from());
          graph::StatePtr_t to = transition->to();
          graph::StatePtr_t from2 = from, to2 = to;

          Configuration_t q0 = path->initial (),
                          q1 = path->end ();
          const bool src_contains_q0 = from->contains (q0);
          const bool dst_contains_q0 = to  ->contains (q0);
          const bool src_contains_q1 = from->contains (q1);
          const bool dst_contains_q1 = to  ->contains (q1);

          bool use_direct  = src_contains_q0 && dst_contains_q1;
          bool use_reverse = src_contains_q1 && dst_contains_q0;
          if (use_direct && use_reverse) {
            if (i == 0 || stateOfStart == from)
              use_reverse = false;
            else if (stateOfStart == to)
              use_direct = false;
            else if (stateOfStart) {
              if (stateOfStart->contains(q0))
                use_reverse = false;
              else
                use_direct = false; // assumes stateOfStart->contains(q0)
            } else
              use_reverse = false; // default if we don't know what to do...
          }
          if (use_direct) {
            // Nominal case
            if (transition->state() != to) {
              constrainEndIntoState (path, i, splines[i], transition->to(), lc);
            }
            stateOfStart = to;
          } else if (use_reverse) {
            // Reversed nominal case
            if (transition->state() != from) {
              constrainEndIntoState (path, i, splines[i], from, lc);
            }
            stateOfStart = from;
          } else {
            if (src_contains_q0) { // q1 must stay in state
              to2 = transition->state();
              stateOfStart = to;
            } else if (dst_contains_q0) { // q1 must stay in state
              from2 = transition->state();
              stateOfStart = from;
            } else if (src_contains_q1) { // q1 must stay in src
              to2 = transition->state();
              stateOfStart = from;
              if (transition->state() != from) {
                constrainEndIntoState (path, i, splines[i], from, lc);
              }
            } else if (dst_contains_q1) { // q1 must stay in dst
              from2 = transition->state();
              stateOfStart = to;
              if (transition->state() != to) {
                constrainEndIntoState (path, i, splines[i], to, lc);
              }
            } else {
              // q0 and q1 are in state. We add no constraint.
              hppDout (warning, "Add no constraint for this path.");
              from2 = transition->state();
              to2 = transition->state();
              stateOfStart.reset();
            }
          }
          if (zeroDerivative) {
            if (   !(use_reverse && src_contains_q0 && src_contains_q1)
                && !(use_direct  && dst_contains_q0 && dst_contains_q1)
                && from2 != to2                         ) {
              constraintDerivativesAtEndOfSpline (i, splines[i], lc);
            }
          }
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
        Eigen::RowBlockIndices select =
          this->computeActiveParameters (path, set->configProjector()->solver(), guessThreshold, true);

        const size_type rDof = this->robot_->numberDof(),
                        col  = idxSpline * Spline::NbCoeffs * rDof,
                        row = lc.J.rows(),
                        nOutVar = select.nbIndices();

        // Add nOutVar constraints
        lc.addRows(nOutVar);
        matrix_t I = select.rview(matrix_t::Identity(rDof, rDof));
        for (size_type k = 0; k < Spline::NbCoeffs; ++k)
          lc.J.block  (row, col + k * rDof, nOutVar, rDof) = B1(k) * I;
        lc.b.segment(row, nOutVar) = select.rview(spline->parameters().transpose() * B1);

        assert ((lc.J.block(row, col, nOutVar, rDof * Spline::NbCoeffs) * spline->rowParameters())
            .isApprox(lc.b.segment(row, nOutVar)));
      }

      template <int _PB, int _SO>
      void SplineGradientBased<_PB, _SO>::constraintDerivativesAtEndOfSpline
      (const size_type& idxSpline, const SplinePtr_t& spline,
       LinearConstraint& lc) const
      {
        typename Spline::BasisFunctionVector_t B1;
        spline->basisFunctionDerivative(1, 1, B1);

        // ConstraintSetPtr_t set = state->configConstraint();
        // value_type guessThreshold = this->problem().getParameter ("SplineGradientBased/guessThreshold", value_type(-1));
        // Eigen::RowBlockIndices select =
          // this->computeActiveParameters (path, set->configProjector()->solver(), guessThreshold);

        const size_type rDof = this->robot_->numberDof(),
                        col  = idxSpline * Spline::NbCoeffs * rDof,
                        row = lc.J.rows(),
                        // nOutVar = select.nbIndices();
                        nOutVar = rDof;

        // Add nOutVar constraints
        lc.addRows(nOutVar);
        // matrix_t I = select.rview(matrix_t::Identity(rDof, rDof));
        matrix_t I (matrix_t::Identity(rDof, rDof));
        for (size_type k = 0; k < Spline::NbCoeffs; ++k)
          lc.J.block  (row, col + k * rDof, nOutVar, rDof) = B1(k) * I;
        lc.b.segment(row, nOutVar).setZero();

        if (!(lc.J.block(row, col, nOutVar, rDof * Spline::NbCoeffs) * spline->rowParameters())
            .isApprox(lc.b.segment(row, nOutVar)))
        {
          hppDout (error, "The velocity should already be zero:\n"
              << (lc.J.block(row, col, nOutVar, rDof * Spline::NbCoeffs) * spline->rowParameters()).transpose()
              );
        }
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
