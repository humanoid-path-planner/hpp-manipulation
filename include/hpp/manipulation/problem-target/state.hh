//
// Copyright (c) 2016 CNRS
// Authors: Joseph Mirabel
//
// This file is part of hpp-manipulation
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
// hpp-manipulation  If not, see
// <http://www.gnu.org/licenses/>.

#ifndef HPP_MANIPULATION_PROBLEM_TARGET_STATE_HH
# define HPP_MANIPULATION_PROBLEM_TARGET_STATE_HH

# include <hpp/core/problem-target.hh>

# include <hpp/core/fwd.hh>
# include <hpp/manipulation/fwd.hh>
# include <hpp/manipulation/graph/fwd.hh>
# include <hpp/manipulation/config.hh>

namespace hpp {
  namespace manipulation {
    namespace problemTarget {
      /// \addtogroup path_planning
      /// \{

      /// State
      ///
      /// This class defines a goal using state of the constraint graph.
      class HPP_MANIPULATION_DLLAPI State : public core::ProblemTarget {
        public:
          static StatePtr_t create (const core::ProblemPtr_t& problem);

          /// Check if the problem target is well specified.
          void check (const core::RoadmapPtr_t& roadmap) const;

          /// Check whether the problem is solved.
          bool reached (const core::RoadmapPtr_t& roadmap) const;

          core::PathVectorPtr_t computePath(const core::RoadmapPtr_t& roadmap) const;

          void target (const graph::StatePtr_t& state)
          {
            state_ = state;
          }

        protected:
          /// Constructor
          State (const core::ProblemPtr_t& problem)
            : ProblemTarget (problem)
          {}

        private:
          graph::StatePtr_t state_;
      }; // class State
      /// \}
    } // namespace problemTarget
  } //   namespace manipulation
} // namespace hpp
#endif // HPP_MANIPULATION_PROBLEM_TARGET_STATE_HH
