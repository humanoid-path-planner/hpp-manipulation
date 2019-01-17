// Copyright (c) 2018, Joseph Mirabel
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
