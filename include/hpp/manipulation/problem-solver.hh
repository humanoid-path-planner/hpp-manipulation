// Copyright (c) 2014 CNRS
// Author: Florent Lamiraux, Joseph Mirabel
//
// This file is part of hpp-manipulation-corba.
// hpp-manipulation-corba is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-manipulation-corba is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-manipulation-corba.  If not, see
// <http://www.gnu.org/licenses/>.

#ifndef HPP_MANIPULATION_PROBLEM_SOLVER_HH
# define HPP_MANIPULATION_PROBLEM_SOLVER_HH

# include <map>
# include <hpp/pinocchio/device.hh>
# include <hpp/core/problem-solver.hh>
# include <hpp/core/container.hh>
# include "hpp/manipulation/constraint-set.hh"
# include "hpp/manipulation/fwd.hh"
# include "hpp/manipulation/deprecated.hh"
# include "hpp/manipulation/device.hh"
# include "hpp/manipulation/graph/fwd.hh"

namespace hpp {
  namespace manipulation {
    class HPP_MANIPULATION_DLLAPI ProblemSolver :
      public core::ProblemSolver
    {
      public:
        typedef core::ProblemSolver parent_t;
        typedef std::vector <std::string> Names_t;

        /// Destructor
        virtual ~ProblemSolver ()
        {}

        ProblemSolver ();

        static ProblemSolverPtr_t create ();

        /// Set robot
        /// Check that robot is of type hpp::manipulation::Device
        virtual void robot (const core::DevicePtr_t& robot)
        {
          robot_ = HPP_DYNAMIC_PTR_CAST (Device, robot);
          assert (robot_);
          parent_t::robot (robot);
        }

        /// Get robot
        const DevicePtr_t& robot () const
        {
          return robot_;
        }

        /// \name Constraint graph
        /// \{

        /// Set the constraint graph
        void constraintGraph (const std::string& graph);

        /// Get the constraint graph
        graph::GraphPtr_t constraintGraph () const;

        /// Should be called before any call on the graph is made.
        void initConstraintGraph ();
        /// \}

	/// Create placement constraint
	/// \param name name of the placement constraint,
	/// \param triangleName name of the first list of triangles,
	/// \param envContactName name of the second list of triangles.
        /// \param margin see hpp::constraints::ConvexShapeContact::setNormalMargin
	/// 
	void createPlacementConstraint (const std::string& name,
					const StringList_t& surface1,
					const StringList_t& surface2,
                                        const value_type& margin = 1e-4);

	/// Create pre-placement constraint
	/// \param name name of the placement constraint,
	/// \param triangleName name of the first list of triangles,
	/// \param envContactName name of the second list of triangles.
	/// \param width approaching distance.
        /// \param margin see hpp::constraints::ConvexShapeContact::setNormalMargin
	/// 
	void createPrePlacementConstraint (const std::string& name,
					   const StringList_t& surface1,
					   const StringList_t& surface2,
                                           const value_type& width,
                                           const value_type& margin = 1e-4);

	/// Create the grasp constraint and its complement
	/// \param name name of the grasp constraint,
	/// \param gripper gripper's name
	/// \param handle handle's name
	/// 
        /// Two constraints are created:
        /// - "name" corresponds to the grasp constraint.
        /// - "name/complement" corresponds to the complement.
        void createGraspConstraint (const std::string& name,
                                    const std::string& gripper,
                                    const std::string& handle);

	/// Create pre-grasp constraint
	/// \param name name of the grasp constraint,
	/// \param gripper gripper's name
	/// \param handle handle's name
	/// 
        void createPreGraspConstraint (const std::string& name,
                                       const std::string& gripper,
                                       const std::string& handle);

        virtual void pathValidationType (const std::string& type,
                                         const value_type& tolerance);

        /// Create a new problem.
        virtual void resetProblem ();

        /// Create a new Roadmap
        virtual void resetRoadmap ();

        /// Get pointer to problem
        ProblemPtr_t problem () const
        {
          return problem_;
        }

        void setTargetState (const graph::StatePtr_t state);

        core::Container <graph::GraphPtr_t> graphs;

        ConstraintsAndComplements_t constraintsAndComplements;

      protected:
        virtual void initializeProblem (ProblemPtr_t problem);

      private:
        /// Keep track of the created components in order to retrieve them
        /// easily.
        graph::GraphComponents_t components_;

        DevicePtr_t robot_;
        /// The pointer should point to the same object as core::Problem.
        ProblemPtr_t problem_;
        graph::GraphPtr_t constraintGraph_;
    }; // class ProblemSolver
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_PROBLEM_SOLVER_HH
