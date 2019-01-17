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

#ifndef HPP_MANIPULATION_PATH_OPTIMIZATION_RANDOM_SHORTCUT_HH
# define HPP_MANIPULATION_PATH_OPTIMIZATION_RANDOM_SHORTCUT_HH

#include <hpp/core/path-optimization/random-shortcut.hh>

#include <hpp/manipulation/fwd.hh>
#include <hpp/manipulation/config.hh>

namespace hpp {
  namespace manipulation {
    /// \addtogroup path_optimization
    /// \{
    namespace pathOptimization {
      HPP_PREDEF_CLASS (RandomShortcut);
      typedef boost::shared_ptr<RandomShortcut> RandomShortcutPtr_t;

      class HPP_MANIPULATION_DLLAPI RandomShortcut :
        public core::pathOptimization::RandomShortcut
      {
        public:
          /// Return shared pointer to new object.
          static RandomShortcutPtr_t create (const core::Problem& problem)
          {
            return RandomShortcutPtr_t (new RandomShortcut (problem));
          }

        protected:
          RandomShortcut (const core::Problem& problem)
            : core::pathOptimization::RandomShortcut (problem)
          {}

          /// Sample times along currentOpt.
          /// t1 and t2 will not be on a path whose transition was short.
          virtual bool shootTimes (const core::PathVectorPtr_t& currentOpt,
              const value_type& t0,
              value_type& t1,
              value_type& t2,
              const value_type& t3);
      }; // class RandomShortcut
    /// \}
    } // namespace pathOptimization
  }  // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_PATH_OPTIMIZATION_RANDOM_SHORTCUT_HH
