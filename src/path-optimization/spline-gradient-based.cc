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

#include <hpp/core/path-optimization/linear-constraint.hh>
#include <hpp/core/steering-method/straight.hh>
#include <hpp/manipulation/constraint-set.hh>
#include <hpp/manipulation/graph/edge.hh>
#include <hpp/manipulation/graph/state.hh>
#include <hpp/manipulation/path-optimization/spline-gradient-based.hh>
#include <hpp/manipulation/problem.hh>

namespace hpp {
namespace manipulation {
namespace pathOptimization {

using core::PathPtr_t;
using core::PathVector;
using core::PathVectorPtr_t;

template <int _PB, int _SO>
SplineGradientBased<_PB, _SO>::SplineGradientBased(
    const ProblemConstPtr_t& problem)
    : Parent_t(problem) {
  this->checkOptimum_ = true;
}

// ----------- Convenience class -------------------------------------- //

// ----------- Resolution steps --------------------------------------- //

template <int _PB, int _SO>
typename SplineGradientBased<_PB, _SO>::Ptr_t
SplineGradientBased<_PB, _SO>::create(const ProblemConstPtr_t& problem) {
  SplineGradientBased* ptr = new SplineGradientBased(problem);
  Ptr_t shPtr(ptr);
  return shPtr;
}

template <int _PB, int _SO>
typename SplineGradientBased<_PB, _SO>::Ptr_t
SplineGradientBased<_PB, _SO>::createFromCore(
    const core::ProblemConstPtr_t& problem) {
  if (!HPP_DYNAMIC_PTR_CAST(const Problem, problem))
    throw std::invalid_argument("This is not a manipulation problem.");
  return create(HPP_STATIC_PTR_CAST(const Problem, problem));
}

template <int _PB, int _SO>
PathVectorPtr_t SplineGradientBased<_PB, _SO>::optimize(
    const PathVectorPtr_t& path) {
  // Get the constraint graph via the problem
  ProblemConstPtr_t mp(HPP_DYNAMIC_PTR_CAST(const Problem, this->problem()));
  assert(mp);
  graph::GraphPtr_t graph(mp->constraintGraph());
  const size_type nq = path->outputSize(), nv = path->outputDerivativeSize();
  PathVectorPtr_t flat = PathVector::create(nq, nv);
  path->flatten(flat);

  // Group consecutive paths by transition.
  std::vector<PathVectorPtr_t> transitions;
  std::vector<graph::EdgePtr_t> edges;
  for (std::size_t i = 0; i < flat->numberPaths(); ++i) {
    PathPtr_t p = flat->pathAtRank(i);
    bool still = p->length() <= 1e-9;
    ConstraintSetPtr_t c =
        HPP_DYNAMIC_PTR_CAST(ConstraintSet, p->constraints());
    if (!c || !c->edge()) {
      if (still) continue;
      throw std::invalid_argument("Path p has no manipulation transition.");
    }
    if (edges.empty() || graph->equivalentEdge(c->edge()) !=
                             graph->equivalentEdge(edges.back())) {
      transitions.push_back(PathVector::create(nq, nv));
      edges.push_back(graph->equivalentEdge(c->edge()));
    }
    // Keep a path without motion only for a transition without motion.
    if (!still && transitions.back()->length() <= 1e-9)
      transitions.back() = PathVector::create(nq, nv);
    if (!still || transitions.back()->numberPaths() == 0)
      transitions.back()->appendPath(p);
  }

  // Smooth each transition and join those within the same state. A
  // transition without motion, such as a release in place, keeps its path
  // and splits the smoothing of the state there.
  std::vector<std::vector<PathVectorPtr_t> > states;
  graph::StatePtr_t previous;
  core::SteeringMethodPtr_t straight =
      core::steeringMethod::Straight::create(this->problem());
  for (std::size_t i = 0; i < transitions.size(); ++i) {
    PathVectorPtr_t transition = transitions[i];
    if (transition->length() <= 1e-9) {
      if (states.empty()) states.emplace_back();
      states.back().push_back(transition);
      continue;
    }
    if (transition->length() < 1e-6)
      throw std::invalid_argument(
          "Path transition is too short for spline timing");
    // Join each path too short for timing with a neighbour: shortcuts can
    // split a short transition into such leaves.
    std::vector<PathPtr_t> leaves;
    for (std::size_t j = 0; j < transition->numberPaths(); ++j)
      leaves.push_back(transition->pathAtRank(j));
    for (std::size_t j = 0; j < leaves.size() && leaves.size() > 1;) {
      if (leaves[j]->length() >= 1e-6) {
        ++j;
        continue;
      }
      if (j + 1 == leaves.size()) --j;
      straight->constraints(leaves[j]->constraints());
      leaves[j] = straight->steer(leaves[j]->initial(), leaves[j + 1]->end());
      if (!leaves[j])
        throw std::runtime_error("Failed to join short leaves of transition " +
                                 edges[i]->name());
      leaves.erase(leaves.begin() + j + 1);
    }
    transition = PathVector::create(nq, nv);
    for (const PathPtr_t& p : leaves) transition->appendPath(p);
    if (edges[i]->state() != previous) {
      states.emplace_back();
      previous = edges[i]->state();
    }
    std::vector<PathVectorPtr_t>& parts = states.back();
    if (parts.empty() || parts.back()->length() <= 1e-9)
      parts.push_back(PathVector::create(nq, nv));
    parts.back()->concatenate(Parent_t::optimize(transition));
  }

  PathVectorPtr_t result = PathVector::create(nq, nv);
  for (const std::vector<PathVectorPtr_t>& parts : states) {
    PathVectorPtr_t state = PathVector::create(nq, nv);
    for (const PathVectorPtr_t& part : parts)
      state->concatenate(part->length() <= 1e-9 ? part
                                                : Parent_t::optimize(part));
    result->appendPath(state);
  }
  return result;
}

template <int _PB, int _SO>
void SplineGradientBased<_PB, _SO>::initializePathValidation(
    const Splines_t& splines) {
  this->validations_.resize(splines.size());
  for (std::size_t i = 0; i < splines.size(); ++i) {
    ConstraintSetPtr_t set =
        HPP_STATIC_PTR_CAST(ConstraintSet, splines[i]->constraints());
    if (set && set->edge())
      this->validations_[i] = set->edge()->pathValidation();
    else
      this->validations_[i] = this->problem()->pathValidation();
  }
}

template <int _PB, int _SO>
void SplineGradientBased<_PB, _SO>::addProblemConstraints(
    const core::PathVectorPtr_t& init, const Splines_t& splines,
    LinearConstraint& lc, SplineOptimizationDatas_t& sods) const {
  assert(init->numberPaths() == splines.size() &&
         sods.size() == splines.size());

  bool zeroDerivative =
      this->problem()
          ->getParameter(
              "SplineGradientBased/zeroDerivativesAtStateIntersection")
          .boolValue();

  const std::size_t last = splines.size() - 1;
  graph::StatePtr_t stateOfStart;
  for (std::size_t i = 0; i < last; ++i) {
    core::PathPtr_t path = init->pathAtRank(i);
    ConstraintSetPtr_t set =
        HPP_STATIC_PTR_CAST(ConstraintSet, splines[i]->constraints());
    if (!set || !set->edge())
      std::invalid_argument(
          "Cannot optimize a path that has not been "
          "generated with a graph.");
    graph::EdgePtr_t transition = set->edge();

    this->addProblemConstraintOnPath(path, i, splines[i], lc, sods[i]);

    // The path should always go through the start and end states of the
    // transition.
    assert(!HPP_DYNAMIC_PTR_CAST(graph::WaypointEdge, transition));
    graph::StatePtr_t from = transition->stateFrom();
    graph::StatePtr_t to = transition->stateTo();
    graph::StatePtr_t from2 = from, to2 = to;

    Configuration_t q0 = path->initial(), q1 = path->end();
    const bool src_contains_q0 = from->contains(q0);
    const bool dst_contains_q0 = to->contains(q0);
    const bool src_contains_q1 = from->contains(q1);
    const bool dst_contains_q1 = to->contains(q1);

    bool use_direct = src_contains_q0 && dst_contains_q1;
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
          use_direct = false;  // assumes stateOfStart->contains(q0)
      } else
        use_reverse = false;  // default if we don't know what to do...
    }
    if (use_direct) {
      // Nominal case
      if (transition->state() != to) {
        constrainEndIntoState(path, i, splines[i], transition->stateTo(), lc);
      }
      stateOfStart = to;
    } else if (use_reverse) {
      // Reversed nominal case
      if (transition->state() != from) {
        constrainEndIntoState(path, i, splines[i], from, lc);
      }
      stateOfStart = from;
    } else {
      if (src_contains_q0) {  // q1 must stay in state
        to2 = transition->state();
        stateOfStart = to;
      } else if (dst_contains_q0) {  // q1 must stay in state
        from2 = transition->state();
        stateOfStart = from;
      } else if (src_contains_q1) {  // q1 must stay in src
        to2 = transition->state();
        stateOfStart = from;
        if (transition->state() != from) {
          constrainEndIntoState(path, i, splines[i], from, lc);
        }
      } else if (dst_contains_q1) {  // q1 must stay in dst
        from2 = transition->state();
        stateOfStart = to;
        if (transition->state() != to) {
          constrainEndIntoState(path, i, splines[i], to, lc);
        }
      } else {
        // q0 and q1 are in state. We add no constraint.
        hppDout(warning, "Add no constraint for this path.");
        from2 = transition->state();
        to2 = transition->state();
        stateOfStart.reset();
      }
    }
    if (zeroDerivative) {
      if (!(use_reverse && src_contains_q0 && src_contains_q1) &&
          !(use_direct && dst_contains_q0 && dst_contains_q1) && from2 != to2) {
        constraintDerivativesAtEndOfSpline(i, splines[i], lc);
      }
    }
  }
  this->addProblemConstraintOnPath(init->pathAtRank(last), last, splines[last],
                                   lc, sods[last]);
}

template <int _PB, int _SO>
void SplineGradientBased<_PB, _SO>::constrainEndIntoState(
    const core::PathPtr_t& path, const size_type& idxSpline,
    const SplinePtr_t& spline, const graph::StatePtr_t state,
    LinearConstraint& lc) const {
  typename Spline::BasisFunctionVector_t B1;
  spline->basisFunctionDerivative(0, 1, B1);
  // TODO Should we add zero velocity sometimes ?

  ConstraintSetPtr_t set = state->configConstraint();
  value_type guessThreshold =
      this->problem()
          ->getParameter("SplineGradientBased/guessThreshold")
          .floatValue();
  Eigen::RowBlockIndices select = this->computeActiveParameters(
      path, set->configProjector()->solver(), guessThreshold, true);
  hppDout(info,
          "End of path " << idxSpline << ": do not change this dof " << select);
  hppDout(info, "End of path " << idxSpline << ": state " << state->name());

  const size_type rDof = this->robot_->numberDof(),
                  col = idxSpline * Spline::NbCoeffs * rDof, row = lc.J.rows(),
                  nOutVar = select.nbIndices();

  // Add nOutVar constraints
  lc.addRows(nOutVar);
  matrix_t I = select.rview(matrix_t::Identity(rDof, rDof));
  for (size_type k = 0; k < Spline::NbCoeffs; ++k)
    lc.setBlock(row, col + k * rDof, B1(k) * I);
  lc.b.segment(row, nOutVar) =
      select.rview(spline->parameters().transpose() * B1);

  assert((lc.J.block(row, col, nOutVar, rDof * Spline::NbCoeffs) *
          spline->rowParameters())
             .isApprox(lc.b.segment(row, nOutVar)));
}

template <int _PB, int _SO>
void SplineGradientBased<_PB, _SO>::constraintDerivativesAtEndOfSpline(
    const size_type& idxSpline, const SplinePtr_t& spline,
    LinearConstraint& lc) const {
  typename Spline::BasisFunctionVector_t B1;
  spline->basisFunctionDerivative(1, 1, B1);

  // ConstraintSetPtr_t set = state->configConstraint();
  // value_type guessThreshold = this->problem().getParameter
  // ("SplineGradientBased/guessThreshold").floatValue(); Eigen::RowBlockIndices
  // select = this->computeActiveParameters (path,
  // set->configProjector()->solver(), guessThreshold);

  const size_type rDof = this->robot_->numberDof(),
                  col = idxSpline * Spline::NbCoeffs * rDof, row = lc.J.rows(),
                  // nOutVar = select.nbIndices();
      nOutVar = rDof;

  // Add nOutVar constraints
  lc.addRows(nOutVar);
  // matrix_t I = select.rview(matrix_t::Identity(rDof, rDof));
  matrix_t I(matrix_t::Identity(rDof, rDof));
  for (size_type k = 0; k < Spline::NbCoeffs; ++k)
    lc.setBlock(row, col + k * rDof, B1(k) * I);
  lc.b.segment(row, nOutVar).setZero();

  if (!(lc.J.block(row, col, nOutVar, rDof * Spline::NbCoeffs) *
        spline->rowParameters())
           .isApprox(lc.b.segment(row, nOutVar))) {
    hppDout(error,
            "The velocity should already be zero:\n"
                << (lc.J.block(row, col, nOutVar, rDof * Spline::NbCoeffs) *
                    spline->rowParameters())
                       .transpose());
  }
}

// ----------- Optimize ----------------------------------------------- //

// ----------- Convenience functions ---------------------------------- //

// ----------- Instanciate -------------------------------------------- //

template class SplineGradientBased<core::path::BernsteinBasis,
                                   1>;  // equivalent to StraightPath
template class SplineGradientBased<core::path::BernsteinBasis, 3>;
template class SplineGradientBased<core::path::BernsteinBasis, 5>;
template class SplineGradientBased<core::path::BernsteinBasis, 7>;

using core::Parameter;
using core::ParameterDescription;

HPP_START_PARAMETER_DECLARATION(SplineGradientBased)
core::Problem::declareParameter(ParameterDescription(
    Parameter::BOOL, "SplineGradientBased/zeroDerivativesAtStateIntersection",
    "Whether we should enforce a null velocity at each control point where the "
    "state before and after is different.",
    Parameter(false)));
HPP_END_PARAMETER_DECLARATION(SplineGradientBased)
}  // namespace pathOptimization
}  // namespace manipulation
}  // namespace hpp
