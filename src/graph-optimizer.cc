// Copyright (c) 2015, Joseph Mirabel
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

#include <hpp/core/config-projector.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/core/path.hh>
#include <hpp/core/problem.hh>
#include <hpp/manipulation/constraint-set.hh>
#include <hpp/manipulation/graph-optimizer.hh>
#include <hpp/manipulation/graph-path-validation.hh>
#include <hpp/manipulation/graph/edge.hh>

namespace hpp {
namespace manipulation {
PathVectorPtr_t GraphOptimizer::optimize(const PathVectorPtr_t& path) {
  PathVectorPtr_t opted = PathVector::create(path->outputSize(),
                                             path->outputDerivativeSize()),
                  expanded = PathVector::create(path->outputSize(),
                                                path->outputDerivativeSize()),
                  toConcat;
  GraphPathValidationPtr_t gpv = HPP_DYNAMIC_PTR_CAST(
      GraphPathValidation, this->problem()->pathValidation());

  path->flatten(expanded);
  ConstraintSetPtr_t c;
  for (std::size_t i_s = 0; i_s < expanded->numberPaths();) {
    PathVectorPtr_t toOpt =
        PathVector::create(path->outputSize(), path->outputDerivativeSize());
    PathPtr_t current = expanded->pathAtRank(i_s);
    toOpt->appendPath(current);
    graph::EdgePtr_t edge;
    c = HPP_DYNAMIC_PTR_CAST(ConstraintSet, current->constraints());
    if (c) edge = c->edge();
    bool isShort = edge && edge->isShort();
    std::size_t i_e = i_s + 1;
    for (; i_e < expanded->numberPaths(); ++i_e) {
      current = expanded->pathAtRank(i_e);
      c = HPP_DYNAMIC_PTR_CAST(ConstraintSet, current->constraints());
      if (!c && edge) {
        hppDout(info, "No manipulation::ConstraintSet");
        break;
      }
      if (c && edge->state() != c->edge()->state()) break;
      if (isShort !=
          c->edge()->isShort())  // We do not optimize edges marked as short
        break;
      toOpt->appendPath(current);
    }
    hppDout(info, "Edge name: " << edge->name());
    if (isShort)
      toConcat = toOpt;
    else {
      core::ProblemPtr_t p = core::Problem::create(problem()->robot());
      p->distance(problem()->distance());
      // It should be ok to use the path validation of each edge because it
      // corresponds to the global path validation minus the collision pairs
      // disabled using the edge constraint.
      // p.pathValidation(gpv->innerValidation());
      p->pathProjector(problem()->pathProjector());
      p->steeringMethod(edge->steeringMethod()->copy());
      p->constraints(p->steeringMethod()->constraints());
      p->constraints()->configProjector()->rightHandSideFromConfig(
          toOpt->initial());
      p->pathValidation(edge->pathValidation());
      pathOptimizer_ = factory_(p);
      toConcat = pathOptimizer_->optimize(toOpt);
    }
    i_s = i_e;
    opted->concatenate(toConcat);
  }
  pathOptimizer_.reset();
  return opted;
}
}  // namespace manipulation
}  // namespace hpp
