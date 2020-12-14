// Copyright (c) 2016, Joseph Mirabel
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

#include <hpp/manipulation/path-optimization/small-steps.hh>

#include <hpp/core/problem.hh>
#include <hpp/core/path-vector.hh>

#include <hpp/wholebody-step/small-steps.hh>

#include <hpp/manipulation/graph/edge.hh>
#include <hpp/manipulation/constraint-set.hh>
#include <hpp/manipulation/graph-path-validation.hh>

namespace hpp {
  namespace manipulation {
    namespace pathOptimization {
      PathVectorPtr_t SmallSteps::optimize (const PathVectorPtr_t& path)
      {
        PathVectorPtr_t
          opted = PathVector::create (path->outputSize(),
                                      path->outputDerivativeSize()),
          flat =  PathVector::create (path->outputSize(),
                                      path->outputDerivativeSize()),
          toConcat;
        path->flatten (flat);

        GraphPathValidationPtr_t gpv(HPP_DYNAMIC_PTR_CAST(GraphPathValidation,
					    this->problem()->pathValidation()));
        const_cast<core::Problem&>(*this->problem()).pathValidation
	  (gpv->innerValidation());

        wholebodyStep::SmallStepsPtr_t stepPtr
          (wholebodyStep::SmallSteps::create(problem()));
        wholebodyStep::SmallSteps& step (*stepPtr);
        step.leftHand_.active = true;

        ConstraintSetPtr_t c;
        for (std::size_t i_s = 0; i_s < flat->numberPaths ();) {
          PathVectorPtr_t toOpt = PathVector::create (
              path->outputSize(), path->outputDerivativeSize()); 
          PathPtr_t current = flat->pathAtRank (i_s);
          toOpt->appendPath (current);
          graph::EdgePtr_t edge;
          c = HPP_DYNAMIC_PTR_CAST (ConstraintSet, current->constraints ());
          if (c) edge = c->edge ();
          std::size_t i_e = i_s + 1;
          for (; i_e < flat->numberPaths (); ++i_e) {
            current = flat->pathAtRank (i_e);
            c = HPP_DYNAMIC_PTR_CAST (ConstraintSet, current->constraints ());
            if (!c && edge) break;
            if (c && edge->state() != c->edge ()->state()) break;
            toOpt->appendPath (current);
          }
          toConcat = step.optimize (toOpt);
          i_s = i_e;
          opted->concatenate (toConcat);
        }

        const_cast<core::Problem&>(*this->problem ()).pathValidation (gpv);
        return opted;
      }
    } // namespace pathOptimization
  } // namespace manipulation
} // namespace hpp
