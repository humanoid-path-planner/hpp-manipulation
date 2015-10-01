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

#include <hpp/manipulation/graph-node-optimizer.hh>

#include <hpp/core/steering-method-straight.hh>

namespace hpp {
  namespace manipulation {
    GraphNodeOptimizerPtr_t GraphNodeOptimizer::create
      (const core::Problem& problem)
    {
      GraphNodeOptimizer* ptr = new GraphNodeOptimizer (problem);
      return GraphNodeOptimizerPtr_t (ptr);
    }

    PathVectorPtr_t GraphNodeOptimizer::optimize (const PathVectorPtr_t& path)
    {
      core::Problem& p = const_cast <core::Problem&> (this->problem ());
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
