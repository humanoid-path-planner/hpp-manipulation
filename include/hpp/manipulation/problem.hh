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

# include "hpp/manipulation/robot.hh"
# include "hpp/manipulation/graph/graph.hh"
# include "hpp/manipulation/graph-path-validation.hh"
# include "hpp/manipulation/graph-steering-method.hh"
# include "hpp/manipulation/fwd.hh"

namespace hpp {
  namespace manipulation {
    class HPP_MANIPULATION_DLLAPI Problem : public core::Problem
    {
      public:
        typedef core::Problem Parent;

        /// Constructor
        Problem (RobotPtr_t robot) : Parent (robot),
          graph_(), steeringMethod_ (new GraphSteeringMethod (robot))
        {
          Parent::steeringMethod (steeringMethod_);
        }

        /// Set the graph of constraints
        void constraintGraph (const graph::GraphPtr_t& graph)
        {
          graph_ = graph;
          steeringMethod_->constraintGraph (graph);
          if (pathValidation ())
            pathValidation ()->constraintGraph (graph);
        }

        /// Get the graph of constraints
        graph::GraphPtr_t constraintGraph () const
        {
          return graph_;
        }

        /// Check whether the problem is well formulated.
        virtual void checkProblem () const
        {
          core::Problem::checkProblem ();
          if (!graph_)
            throw std::runtime_error ("No graph in the problem.");
          if (!pathValidation ())
            throw std::runtime_error ("No GraphPathValidation in the problem.");
          if (!steeringMethod ())
            throw std::runtime_error ("No GraphSteeringMethod in the problem.");
        }

        /// Get the path validation as a GraphPathValidation
        GraphPathValidationPtr_t pathValidation () const
        {
          return HPP_DYNAMIC_PTR_CAST (GraphPathValidation, Parent::pathValidation());
        }

        /// Get the steering method as a GraphSteeringMethod
        GraphSteeringMethodPtr_t steeringMethod () const
        {
          return steeringMethod_;
        }

      private:
        /// The graph of constraints
        graph::GraphPtr_t graph_;
        /// Steering method
        GraphSteeringMethodPtr_t steeringMethod_;
    };
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_PROBLEM_HH
