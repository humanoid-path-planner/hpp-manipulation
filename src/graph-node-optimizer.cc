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

#include <hpp/manipulation/graph-node-optimizer.hh>

#include <hpp/core/steering-method/straight.hh>

namespace hpp {
  namespace manipulation {
    GraphNodeOptimizerPtr_t GraphNodeOptimizer::create
      (const core::ProblemConstPtr_t& problem)
    {
      GraphNodeOptimizer* ptr = new GraphNodeOptimizer (problem);
      return GraphNodeOptimizerPtr_t (ptr);
    }

    PathVectorPtr_t GraphNodeOptimizer::optimize (const PathVectorPtr_t& path)
    {
      core::ProblemPtr_t p = const_cast <core::ProblemPtr_t> (this->problem ());
      core::SteeringMethodPtr_t sm = p.steeringMethod ();

      /// Start by flattening the path
      PathVectorPtr_t flat = PathVector::create
        (path->outputSize(), path->outputDerivativeSize()),
      path->flatten (flat);

      PathVectorPtr_t opted = PathVector::create
        (path->outputSize(), path->outputDerivativeSize()),
        toConcat;

      ConstraintSetPtr_t c;
      for (std::size_t i_s = 0; i_s < flat->numberPaths ();) {
        PathPtr_t p = flat->pathAtRank (i_s);
        PathPtr_t newp = (*sm) (p->initial (), p->end ());
        if (!newp)
          throw std::runtime_error ("It should not be a problem to recompute "
              "a path...");
        opted->appendPath (newp);
      }
      return opted;
    }
  } // namespace manipulation
} // namespace hpp
