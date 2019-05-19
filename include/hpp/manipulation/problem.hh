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
        SteeringMethodPtr_t steeringMethod () const;

        void steeringMethod (core::SteeringMethodPtr_t sm)
        {
          Parent::steeringMethod (sm);
        }

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
