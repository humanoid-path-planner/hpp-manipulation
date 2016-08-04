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

#ifndef HPP_MANIPULATION_PATHOPTIMIZATION_KEYPOINTS_HH
#define HPP_MANIPULATION_PATHOPTIMIZATION_KEYPOINTS_HH

# include <hpp/core/path-optimizer.hh>
# include <hpp/core/path-optimizer.hh>

# include <hpp/manipulation/fwd.hh>
# include <hpp/manipulation/config.hh>
# include <hpp/manipulation/problem.hh>
# include <hpp/manipulation/graph/fwd.hh>

namespace hpp {
  namespace manipulation {
    namespace pathOptimization {
      using hpp::core::Path;
      using hpp::core::PathPtr_t;
      using hpp::core::PathVector;
      using hpp::core::PathVectorPtr_t;
      /// \addtogroup path_optimization
      /// \{

      class HPP_MANIPULATION_DLLAPI Keypoints : public core::PathOptimizer
      {
        public:
          static KeypointsPtr_t create (const core::Problem& problem) {
            const Problem& p = dynamic_cast <const Problem&> (problem);
            return KeypointsPtr_t (new Keypoints (p));
          }

          virtual PathVectorPtr_t optimize (const PathVectorPtr_t& path);

        protected:
          /// Constructor
          Keypoints (const Problem& problem) :
            PathOptimizer (problem), problem_ (problem) {}

        private:
          const Problem& problem_;
          struct InterKeypointPath {
            PathVectorPtr_t path;
            bool isShort;
            graph::EdgePtr_t edge;
          };
          typedef std::vector<InterKeypointPath> IKPvector_t;

          IKPvector_t split (PathVectorPtr_t path) const;

          PathVectorPtr_t shorten (const IKPvector_t& paths,
              std::size_t i1, std::size_t i2) const;

          IKPvector_t replaceInPath(const IKPvector_t& input,
              const PathVectorPtr_t& shortcut,
              std::size_t i1, std::size_t i2) const;
      };

      /// \}

    } // namespace pathOptimization
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_PATHOPTIMIZATION_KEYPOINTS_HH
