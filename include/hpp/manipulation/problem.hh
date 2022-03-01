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

#ifndef HPP_MANIPULATION_PROBLEM_HH
# define HPP_MANIPULATION_PROBLEM_HH

# include <hpp/core/problem.hh>
# include <hpp/core/problem-solver.hh> // PathValidationBuilder_t

# include <hpp/manipulation/fwd.hh>
# include <hpp/manipulation/device.hh>
# include <hpp/manipulation/graph/fwd.hh>

namespace hpp {
  namespace manipulation {
    /// \addtogroup path_planning
    /// \{

    class HPP_MANIPULATION_DLLAPI Problem : public core::Problem
    {
      public:
        typedef core::Problem Parent;

        /// Constructor
        static ProblemPtr_t create (DevicePtr_t robot);

        /// Set the graph of constraints
        void constraintGraph (const graph::GraphPtr_t& graph);

        /// Get the graph of constraints
        graph::GraphPtr_t constraintGraph () const
        {
          return graph_;
        }

        /// Check whether the problem is well formulated.
        virtual void checkProblem () const;

        /// Expose parent method.
        PathValidationPtr_t pathValidation () const;

        /// \param pathValidation if of type GraphPathValidation, sets 
        ///        its constraint graph to Problem::constraintGraph()
        void pathValidation (const PathValidationPtr_t& pathValidation);

        /// Get the steering method as a SteeringMethod
        SteeringMethodPtr_t manipulationSteeringMethod () const;

        /// Build a new path validation
        /// \note Current obstacles are added to the created object.
        /// \todo Keep a pointer to this value to update it when a new obstacle
        /// is added.
        PathValidationPtr_t pathValidationFactory () const;

        void setPathValidationFactory (
            const core::PathValidationBuilder_t& factory,
            const value_type& tol);

      protected:
        /// Constructor
        Problem (DevicePtr_t robot);

        void init (ProblemWkPtr_t wkPtr);

      private:
        ProblemWkPtr_t wkPtr_;

        /// The graph of constraints
        graph::GraphPtr_t graph_;

        core::PathValidationBuilder_t pvFactory_;
        value_type pvTol_;
    }; // class Problem
    /// \}
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_PROBLEM_HH
