//
// Copyright (c) 2015 CNRS
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

#ifndef HPP_MANIPULATION_PATH_OPTIMIZATION_CONFIG_OPTIMIZATION_HH
# define HPP_MANIPULATION_PATH_OPTIMIZATION_CONFIG_OPTIMIZATION_HH

# include <hpp/core/path-optimization/config-optimization.hh>

# include <hpp/manipulation/fwd.hh>

namespace hpp {
  namespace manipulation {
    namespace pathOptimization {
      /// \addtogroup path_optimization
      /// \{

      /// Optimize the waypoints of the path and optionally add the
      /// constraint::ConfigurationConstraint to the ConfigProjector of the
      /// path.
      ///
      /// See Parameters for information on how to tune the algorithm.
      ///
      /// \note The optimizer assumes that the input path is a vector of optimal
      ///       paths for the distance function.
      struct ConfigOptimizationTraits
        : core::pathOptimization::ConfigOptimizationTraits {
          typedef core::PathPtr_t PathPtr_t;

          static size_type numberOfPass () { return 10; }

          static size_type numberOfIterations () { return 1; }

          static ConfigProjectorPtr_t getConfigProjector
            (const PathPtr_t& before, const PathPtr_t& after, bool& reverse);
        };
      /// \}
    } // namespace pathOptimization
  } // namespace manipulation
} // namespace hpp
#endif // HPP_MANIPULATION_PATH_OPTIMIZATION_CONFIG_OPTIMIZATION_HH
