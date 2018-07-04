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

#ifndef HPP_MANIPULATION_PATHOPTIMIZATION_ENFORCE_TRANSITION_SEMANTIC_HH
#define HPP_MANIPULATION_PATHOPTIMIZATION_ENFORCE_TRANSITION_SEMANTIC_HH

# include <hpp/core/path-optimizer.hh>

# include <hpp/manipulation/problem.hh>

namespace hpp {
  namespace manipulation {
    namespace pathOptimization {
      using hpp::core::Path;
      using hpp::core::PathPtr_t;
      using hpp::core::PathVector;
      /// \addtogroup path_optimization
      /// \{

      class HPP_MANIPULATION_DLLAPI EnforceTransitionSemantic : public core::PathOptimizer
      {
        public:
          typedef hpp::core::PathVectorPtr_t PathVectorPtr_t;
          typedef boost::shared_ptr<EnforceTransitionSemantic> Ptr_t;

          static Ptr_t create (const core::Problem& problem) {
            const Problem& p = dynamic_cast <const Problem&> (problem);
            return Ptr_t (new EnforceTransitionSemantic (p));
          }

          virtual PathVectorPtr_t optimize (const PathVectorPtr_t& path);

        protected:
          /// Constructor
          EnforceTransitionSemantic (const Problem& problem) :
            PathOptimizer (problem), problem_ (problem) {}

        private:
          const Problem& problem_;
      };

      /// \}
    } // namespace pathOptimization
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_PATHOPTIMIZATION_ENFORCE_TRANSITION_SEMANTIC_HH
