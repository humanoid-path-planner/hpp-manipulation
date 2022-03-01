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

#include <hpp/manipulation/path-optimization/random-shortcut.hh>

#include <hpp/core/path.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/manipulation/constraint-set.hh>
#include <hpp/manipulation/graph/edge.hh>

namespace hpp {
  namespace manipulation {
    namespace pathOptimization {
      using core::PathPtr_t;
      using core::PathVector;
      using core::PathVectorPtr_t;

      bool isShort (const PathVectorPtr_t& path, const value_type& t)
      {
        value_type localT;
        std::size_t i = path->rankAtParam (t, localT);
        PathPtr_t p = path->pathAtRank (i);
        PathVectorPtr_t pv = HPP_DYNAMIC_PTR_CAST (PathVector, p);
        if (pv) return isShort (pv, localT);
        ConstraintSetPtr_t c = HPP_DYNAMIC_PTR_CAST(ConstraintSet, path->constraints());
        if (c) return c->edge()->isShort();
        return false;
      }

      bool RandomShortcut::shootTimes (const PathVectorPtr_t& currentOpt,
          const value_type& t0,
          value_type& t1,
          value_type& t2,
          const value_type& t3)
      {
        bool ok = false;
        for (int i = 0; i < 5; ++i)
        {
          value_type u2 = (t3-t0) * rand ()/RAND_MAX;
          value_type u1 = (t3-t0) * rand ()/RAND_MAX;
          if (isShort (currentOpt, u1) || isShort (currentOpt, u2))
            continue;
          if (u1 < u2) {
            t1 = t0 + u1;
            t2 = t0 + u2;
          } else {
            t1 = t0 + u2;
            t2 = t0 + u1;
          }
          ok = true;
          break;
        }
        return ok;
      }
    } // namespace pathOptimization
  }  // namespace manipulation
} // namespace hpp
