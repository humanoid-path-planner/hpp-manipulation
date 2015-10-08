// Copyright (c) 2015, Joseph Mirabel
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

#include <hpp/manipulation/graph-optimizer.hh>

#include <hpp/manipulation/graph/edge.hh>

namespace hpp {
  namespace manipulation {
    PathVectorPtr_t GraphOptimizer::optimize (const PathVectorPtr_t& path)
    {
      PathVectorPtr_t opted = PathVector::create
        (path->outputSize(), path->outputDerivativeSize()),
        expanded = PathVector::create
          (path->outputSize(), path->outputDerivativeSize()),
        toConcat;
      unpack (path, expanded);
      ConstraintSetPtr_t c;
      for (std::size_t i_s = 0; i_s < expanded->numberPaths ();) {
        PathVectorPtr_t toOpt = PathVector::create (
            path->outputSize(), path->outputDerivativeSize()); 
        PathPtr_t current = expanded->pathAtRank (i_s);
        toOpt->appendPath (current);
        graph::EdgePtr_t edge;
        c = HPP_DYNAMIC_PTR_CAST (ConstraintSet, current->constraints ());
        if (c) edge = c->edge ();
        std::size_t i_e = i_s + 1;
        for (; i_e < expanded->numberPaths (); ++i_e) {
          current = expanded->pathAtRank (i_e);
          c = HPP_DYNAMIC_PTR_CAST (ConstraintSet, current->constraints ());
          if (!c && edge) break;
          if (c && edge->node() != c->edge ()->node()) break;
          toOpt->appendPath (current);
        }
        pathOptimizer_ = factory_ (this->problem ());
        toConcat = pathOptimizer_->optimize (toOpt);
        i_s = i_e;
        opted->concatenate (*toConcat);
      }
      pathOptimizer_.reset ();
      return opted;
    }

    void GraphOptimizer::unpack (PathVectorPtr_t in, PathVectorPtr_t out)
    {
      for (size_t i = 0; i != in->numberPaths (); i++) {
        PathPtr_t current = in->pathAtRank (i);
        PathVectorPtr_t pv = HPP_DYNAMIC_PTR_CAST (PathVector, current);
        if (pv) {
          unpack (pv, out);
        } else {
          out->appendPath (current);
        }
      }
    }
  } // namespace manipulation
} // namespace hpp
