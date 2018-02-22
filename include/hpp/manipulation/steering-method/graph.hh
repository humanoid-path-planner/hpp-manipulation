// Copyright (c) 2014, LAAS-CNRS
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


#ifndef HPP_MANIPULATION_STEERING_METHOD_GRAPH_HH
# define HPP_MANIPULATION_STEERING_METHOD_GRAPH_HH

# include <hpp/core/problem-solver.hh> // SteeringMethodBuilder_t
# include <hpp/core/steering-method.hh>

# include <hpp/manipulation/config.hh>
# include <hpp/manipulation/fwd.hh>
# include <hpp/manipulation/graph/fwd.hh>
# include <hpp/manipulation/steering-method/fwd.hh>

namespace hpp {
  namespace manipulation {
    /// \addtogroup steering_method
    /// \{
    class HPP_MANIPULATION_DLLAPI SteeringMethod : public core::SteeringMethod
    {
      public:
        const core::SteeringMethodPtr_t& innerSteeringMethod () const
        {
          return steeringMethod_;
        }

        void innerSteeringMethod (const core::SteeringMethodPtr_t& sm)
        {
          steeringMethod_ = sm;
        }

      protected:
        /// Constructor
        SteeringMethod (const Problem& problem);

        /// Copy constructor
        SteeringMethod (const SteeringMethod& other);

        void init (SteeringMethodWkPtr_t weak)
        {
          core::SteeringMethod::init (weak);
        }

        /// A pointer to the manipulation problem
        const Problem& problem_;
        /// The encapsulated steering method
        core::SteeringMethodPtr_t steeringMethod_;
    };

    namespace steeringMethod {
      using core::PathPtr_t;

      class HPP_MANIPULATION_DLLAPI Graph : public SteeringMethod
      {
        typedef core::SteeringMethodBuilder_t SteeringMethodBuilder_t;

        public:
          /// Create instance and return shared pointer
          /// \warning core::Problem will be casted to Problem
          static GraphPtr_t create
            (const core::Problem& problem);

          template <typename T>
            static GraphPtr_t create
            (const core::Problem& problem);

          /// Create instance and return shared pointer
          static GraphPtr_t create (const Problem& problem);

          /// Create copy and return shared pointer
          static GraphPtr_t createCopy
            (const GraphPtr_t& other);

          /// Copy instance and return shared pointer
          virtual core::SteeringMethodPtr_t copy () const
          {
            return createCopy (weak_.lock ());
          }

        protected:
          /// Constructor
          Graph (const Problem& problem);

          /// Copy constructor
          Graph (const Graph&);

          virtual PathPtr_t impl_compute (ConfigurationIn_t q1, ConfigurationIn_t q2) const;

          void init (GraphWkPtr_t weak)
          {
            SteeringMethod::init (weak);
            weak_ = weak;
          }

        private:
          /// Weak pointer to itself
          GraphWkPtr_t weak_;
      };

      template <typename T>
        GraphPtr_t Graph::create
        (const core::Problem& problem)
      {
        GraphPtr_t gsm = Graph::create (problem);
        gsm->innerSteeringMethod (T::create (problem));
        return gsm;
      }
    } // namespace steeringMethod
    /// \}
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_STEERING_METHOD_GRAPH_HH
