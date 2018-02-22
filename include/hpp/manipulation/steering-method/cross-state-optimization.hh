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
# include <hpp/manipulation/graph-steering-method.hh>

namespace hpp {
  namespace manipulation {
    namespace steeringMethod {
      class HPP_MANIPULATION_DLLAPI CrossStateOptimization :
        public SteeringMethod
      {
        public:
          static CrossStateOptimizationPtr_t create (const Problem& problem);

          /// \warning core::Problem will be casted to Problem
          static CrossStateOptimizationPtr_t create
            (const core::Problem& problem);

          template <typename T>
            static CrossStateOptimizationPtr_t create
            (const core::Problem& problem);

          core::SteeringMethodPtr_t copy () const;

        protected:
          CrossStateOptimization (const Problem& problem) :
            SteeringMethod (problem)
          {}

          CrossStateOptimization (const CrossStateOptimization& other) :
            SteeringMethod (other),
            weak_ ()
          {}

          core::PathPtr_t impl_compute (ConfigurationIn_t q1, ConfigurationIn_t q2) const;

          void init (CrossStateOptimizationWkPtr_t weak)
          {
            SteeringMethod::init (weak);
            weak_ = weak;
          }

        private:
          struct GraphSearchData;
          struct OptimizationData;

          /// Step 1 of the algorithm
          /// \return whether the max depth was reached.
          bool findTransitions (GraphSearchData& data) const;

          /// Step 2 of the algorithm
          graph::Edges_t getTransitionList (GraphSearchData& data, const std::size_t& i) const;

          /// Step 3 of the algorithm
          void buildOptimizationProblem (OptimizationData& d, const graph::Edges_t& edges) const;

          core::PathVectorPtr_t buildPath (OptimizationData& d, const graph::Edges_t& edges) const;

          /// Weak pointer to itself
          CrossStateOptimizationWkPtr_t weak_;
      }; // class CrossStateOptimization

      template <typename T>
        CrossStateOptimizationPtr_t CrossStateOptimization::create
        (const core::Problem& problem)
      {
        CrossStateOptimizationPtr_t gsm = CrossStateOptimization::create (problem);
        gsm->innerSteeringMethod (T::create (problem));
        return gsm;
      }
    } // namespace steeringMethod
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_STEERING_METHOD_CROSS_STATE_OPTIMIZATION_HH
