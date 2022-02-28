// Copyright (c) 2015, LAAS-CNRS
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


#ifndef HPP_MANIPULATION_GRAPHOPTIMIZER_HH
# define HPP_MANIPULATION_GRAPHOPTIMIZER_HH

# include <hpp/core/path-optimizer.hh>
# include <hpp/core/problem-solver.hh> // PathOptimizerBuilder_t

# include <hpp/manipulation/fwd.hh>
# include <hpp/manipulation/config.hh>
# include <hpp/manipulation/graph/fwd.hh>

namespace hpp {
  namespace manipulation {
    using hpp::core::Path;
    using hpp::core::PathPtr_t;
    using hpp::core::PathVector;
    using hpp::core::PathVectorPtr_t;

    /// \addtogroup path_optimization
    /// \{

    /// Path optimizer for paths created with the constraint graph
    ///
    /// This class encapsulates another path optimizer class. This optimizer
    /// calls the inner optimizer on every subpaths with the same set of
    /// constraints.
    class HPP_MANIPULATION_DLLAPI GraphOptimizer : public PathOptimizer
    {
      public:
        typedef core::PathOptimizerBuilder_t PathOptimizerBuilder_t;

        template <typename TraitsOrInnerType>
          static GraphOptimizerPtr_t create
	  (const core::ProblemConstPtr_t& problem);

        virtual PathVectorPtr_t optimize (const PathVectorPtr_t& path);

        /// Get the encapsulated optimizer
        const PathOptimizerPtr_t& innerOptimizer ()
        {
          return pathOptimizer_;
        }

      protected:
        /// Constructor
        GraphOptimizer (const core::ProblemConstPtr_t& problem,
			PathOptimizerBuilder_t factory) :
          PathOptimizer (problem), factory_ (factory), pathOptimizer_ ()
        {}

      private:
        PathOptimizerBuilder_t factory_;

        /// The encapsulated PathOptimizer
        PathOptimizerPtr_t pathOptimizer_;
    };
    /// \}

    /// Member function definition
    template <typename TraitsOrInnerType>
      GraphOptimizerPtr_t GraphOptimizer::create
      (const core::ProblemConstPtr_t& problem)
    {
      return GraphOptimizerPtr_t (
          new GraphOptimizer (problem, TraitsOrInnerType::create)
          );
    }

  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_GRAPHOPTIMIZER_HH
