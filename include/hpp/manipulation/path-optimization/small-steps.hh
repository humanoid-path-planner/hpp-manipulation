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

#ifndef HPP_MANIPULATION_PATHOPTIMIZATION_SMALLSTEPS_HH
#define HPP_MANIPULATION_PATHOPTIMIZATION_SMALLSTEPS_HH

#include <hpp/manipulation/fwd.hh>
#include <hpp/manipulation/config.hh>

#include <hpp/core/path-optimizer.hh>

namespace hpp {
  namespace manipulation {
    namespace pathOptimization {
      using hpp::core::Path;
      using hpp::core::PathPtr_t;
      using hpp::core::PathVector;
      using hpp::core::PathVectorPtr_t;

      /// \addtogroup path_optimization
      /// \{

      /// Walking trajectory generator for paths created with the constraint graph
      ///
      /// This class encapsulates hpp::wholebodyStep::SmallSteps.
      class HPP_MANIPULATION_DLLAPI SmallSteps : public PathOptimizer
      {
        public:
          static SmallStepsPtr_t create (const core::Problem& problem)
          {
            SmallSteps* ptr (new SmallSteps (problem));
            return SmallStepsPtr_t (ptr);
          }

          PathVectorPtr_t optimize (const PathVectorPtr_t& path);

        protected:
          /// Constructor
          SmallSteps (const core::Problem& problem) :
            PathOptimizer (problem)
        {}
      };
      /// \}

    } // namespace pathOptimization
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_PATHOPTIMIZATION_SMALLSTEPS_HH
