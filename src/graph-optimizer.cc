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

#include <hpp/core/path.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/config-projector.hh>

#include <hpp/manipulation/constraint-set.hh>
#include <hpp/manipulation/graph/edge.hh>
#include <hpp/manipulation/graph-path-validation.hh>

namespace hpp {
  namespace manipulation {
    PathVectorPtr_t GraphOptimizer::optimize (const PathVectorPtr_t& path)
    {
      PathVectorPtr_t opted = PathVector::create
        (path->outputSize(), path->outputDerivativeSize()),
        expanded = PathVector::create
          (path->outputSize(), path->outputDerivativeSize()),
        toConcat;
      GraphPathValidationPtr_t gpv = HPP_DYNAMIC_PTR_CAST (GraphPathValidation,
              this->problem().pathValidation ());
      core::Problem p (problem().robot());
      p.distance(problem().distance());
      // It should be ok to use the path validation of each edge because it
      // corresponds to the global path validation minus the collision pairs
      // disabled using the edge constraint.
      // p.pathValidation(gpv->innerValidation());
      p.pathProjector(problem().pathProjector());

      path->flatten (expanded);
      ConstraintSetPtr_t c;
      for (std::size_t i_s = 0; i_s < expanded->numberPaths ();) {
        PathVectorPtr_t toOpt = PathVector::create (
            path->outputSize(), path->outputDerivativeSize()); 
        PathPtr_t current = expanded->pathAtRank (i_s);
        toOpt->appendPath (current);
        graph::EdgePtr_t edge;
        c = HPP_DYNAMIC_PTR_CAST (ConstraintSet, current->constraints ());
        if (c) edge = c->edge ();
        bool isShort = edge && edge->isShort();
        std::size_t i_e = i_s + 1;
        for (; i_e < expanded->numberPaths (); ++i_e) {
          current = expanded->pathAtRank (i_e);
          c = HPP_DYNAMIC_PTR_CAST (ConstraintSet, current->constraints ());
          if (!c && edge) {
            hppDout(info, "No manipulation::ConstraintSet");
            break;
          }
          if (c && edge->state() != c->edge ()->state()) break;
          if (isShort != c->edge()->isShort()) // We do not optimize edges marked as short
            break;
          toOpt->appendPath (current);
        }
        hppDout(info, "Edge name: " << edge->name());
        if (isShort)
          toConcat = toOpt;
        else {
          p.constraints(edge->steeringMethod()->constraints());
          p.constraints()->configProjector()->rightHandSideFromConfig(toOpt->initial());
          p.steeringMethod(edge->steeringMethod());
          p.pathValidation(edge->pathValidation());
          pathOptimizer_ = factory_ (p);
          toConcat = pathOptimizer_->optimize (toOpt);
        }
        i_s = i_e;
        opted->concatenate (toConcat);
      }
      pathOptimizer_.reset ();
      return opted;
    }
  } // namespace manipulation
} // namespace hpp
