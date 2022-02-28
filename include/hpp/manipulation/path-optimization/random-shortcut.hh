// Copyright (c) 2018, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.

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
      typedef shared_ptr<RandomShortcut> RandomShortcutPtr_t;

      class HPP_MANIPULATION_DLLAPI RandomShortcut :
        public core::pathOptimization::RandomShortcut
      {
        public:
          /// Return shared pointer to new object.
          static RandomShortcutPtr_t create
	    (const core::ProblemConstPtr_t problem)
          {
            return RandomShortcutPtr_t (new RandomShortcut (problem));
          }

        protected:
          RandomShortcut (const core::ProblemConstPtr_t& problem)
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
