// Copyright (c) 2017, Joseph Mirabel
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

#ifndef HPP_MANIPULATION_STEERING_METHOD_CROSS_STATE_OPTIMIZATION_HH
# define HPP_MANIPULATION_STEERING_METHOD_CROSS_STATE_OPTIMIZATION_HH

# include <hpp/core/steering-method.hh>

# include <hpp/manipulation/config.hh>
# include <hpp/manipulation/fwd.hh>
# include <hpp/manipulation/problem.hh>
# include <hpp/manipulation/steering-method/fwd.hh>

namespace hpp {
  namespace manipulation {
    namespace steeringMethod {
      class HPP_MANIPULATION_DLLAPI CrossStateOptimization :
        public core::SteeringMethod
      {
        public:
          static CrossStateOptimizationPtr_t create (const Problem& problem);

          /// \warning core::Problem will be casted to Problem
          static CrossStateOptimizationPtr_t createFromCore
            (const core::Problem& problem);

          core::SteeringMethodPtr_t copy () const;

        protected:
          CrossStateOptimization (const Problem& problem) :
            core::SteeringMethod (problem),
            problem_ (problem)
          {}

          CrossStateOptimization (const CrossStateOptimization& other) :
            core::SteeringMethod (other),
            problem_ (other.problem_),
            weak_ ()
          {}

          core::PathPtr_t impl_compute (ConfigurationIn_t q1, ConfigurationIn_t q2) const;

          void init (CrossStateOptimizationWkPtr_t weak)
          {
            core::SteeringMethod::init (weak);
            weak_ = weak;
          }

        private:
          struct Data;

          /// Step 1 of the algorithm
          /// \return whether the max depth was reached.
          bool findTransitions (Data& data) const;

          /// Step 2 of the algorithm
          graph::Edges_t getTransitionList (Data& data, const std::size_t& i) const;

          /// A pointer to the problem
          const Problem& problem_;
          /// Weak pointer to itself
          CrossStateOptimizationWkPtr_t weak_;
      }; // class CrossStateOptimization
    } // namespace steeringMethod
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_STEERING_METHOD_CROSS_STATE_OPTIMIZATION_HH
