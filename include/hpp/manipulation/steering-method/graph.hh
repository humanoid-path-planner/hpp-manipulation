// Copyright (c) 2014, LAAS-CNRS
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
        SteeringMethod (const ProblemConstPtr_t& problem);

        /// Copy constructor
        SteeringMethod (const SteeringMethod& other);

        void init (SteeringMethodWkPtr_t weak)
        {
          core::SteeringMethod::init (weak);
        }

        /// A pointer to the manipulation problem
        ProblemConstPtr_t problem_;
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
            (const core::ProblemConstPtr_t& problem);

          template <typename T>
            static GraphPtr_t create
            (const core::ProblemConstPtr_t& problem);

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
          Graph (const ProblemConstPtr_t& problem);

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
        (const core::ProblemConstPtr_t& problem)
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
