// Copyright (c) 2015, LAAS-CNRS
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

#ifndef HPP_MANIPULATION_GRAPH_NODE_OPTIMIZER_HH
# define HPP_MANIPULATION_GRAPH_NODE_OPTIMIZER_HH

# include <hpp/core/path.hh>
# include <hpp/core/path-vector.hh>
# include <hpp/core/path-optimizer.hh>
# include <hpp/core/problem.hh>
# include <hpp/core/problem-solver.hh>

# include <hpp/manipulation/fwd.hh>
# include <hpp/manipulation/graph/fwd.hh>
# include <hpp/manipulation/config.hh>
# include <hpp/manipulation/constraint-set.hh>

namespace hpp {
  namespace manipulation {
    using hpp::core::Path;
    using hpp::core::PathPtr_t;
    using hpp::core::PathVector;
    using hpp::core::PathVectorPtr_t;

    /// \addtogroup path_optimization
    /// \{

    /// Path optimizer that recompute the edge parameter of the constraints
    ///
    /// This class encapsulates another path optimizer class. This optimizer
    /// calls the inner optimizer on every subpaths with the same set of
    /// constraints.
    class HPP_MANIPULATION_DLLAPI GraphNodeOptimizer : public PathOptimizer
    {
      public:
        static GraphNodeOptimizerPtr_t create (const core::Problem& problem);

        virtual PathVectorPtr_t optimize (const PathVectorPtr_t& path);

      protected:
        /// Constructor
        GraphNodeOptimizer (const core::Problem& problem) :
          PathOptimizer (problem)
        {}

      private:
    };
    /// \}
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_GRAPH_NODE_OPTIMIZER_HH
