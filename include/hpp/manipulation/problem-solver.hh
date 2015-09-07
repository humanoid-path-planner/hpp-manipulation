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
# include <hpp/model/device.hh>
# include <hpp/core/problem-solver.hh>
# include "hpp/manipulation/fwd.hh"
# include "hpp/manipulation/deprecated.hh"
# include "hpp/manipulation/device.hh"
# include "hpp/manipulation/container.hh"
# include "hpp/manipulation/graph/fwd.hh"

namespace hpp {
  namespace manipulation {
    class HPP_MANIPULATION_DLLAPI ProblemSolver : public core::ProblemSolver,
    public Container <LockedJointPtr_t>, public Container <JointAndTriangles_t>
    {
      public:
        typedef core::ProblemSolver parent_t;
        typedef std::vector <std::string> Names_t;

        /// Destructor
        virtual ~ProblemSolver ()
        {}

        ProblemSolver () :
          core::ProblemSolver (), robot_ (), problem_ (0x0), graspsMap_()
        {
        }

        static ProblemSolverPtr_t create ();

        /// Set robot
        /// Check that robot is of type hpp::manipulation::Device
        virtual void robot (const DevicePtr_t& robot)
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
        void constraintGraph (const graph::GraphPtr_t& graph);

        /// Get the constraint graph
        graph::GraphPtr_t constraintGraph () const;
        /// \}

        /// Add grasp
        void addGrasp( const DifferentiableFunctionPtr_t& constraint,
            const model::GripperPtr_t& gripper,
            const HandlePtr_t& handle)
        {
          Grasp_t* ptr = new Grasp_t (gripper, handle);
          GraspPtr_t shPtr (ptr);
          graspsMap_[constraint] = shPtr;
        }

        /// get grapsMap
        GraspsMap_t& grasps()
        {
          return graspsMap_;
        }

        /// get graps by name
        ///
        /// return NULL if no grasp named graspName
        GraspPtr_t grasp(const DifferentiableFunctionPtr_t& constraint) const;

        /// Reset constraint set and put back the disable collisions
        /// between gripper and handle
        virtual void resetConstraints ();

        /// Add differential function to the config projector
        /// \param constraintName Name given to config projector if created by
        ///        this method.
        /// \param functionName name of the function as stored in internal map.
        /// Build the config projector if not yet constructed.
        /// If constraint is a graps, deactivate collision between gripper and
        /// object.
        virtual void addFunctionToConfigProjector
          (const std::string& constraintName, const std::string& functionName);

        /// Create a new problem.
        virtual void resetProblem ();

        /// Create a new Roadmap
        virtual void resetRoadmap ();

        /// Get pointer to problem
        ProblemPtr_t problem () const
        {
          return problem_;
        }

        /// Get an element of a container
        template <typename Element>
          const Element& get (const std::string& name) const
        {
          return Container <Element>::get (name);
        }

        /// Add an element to a container
        template <typename Element>
          void add (const std::string& name, const Element& element)
        {
          Container <Element>::add (name, element);
        }

        /// Get the underlying map of a container
        template <typename Element>
          const typename Container<Element>::ElementMap_t& getAll () const
        {
          return Container <Element>::getAll ();
        }

      protected:
        virtual void initializeProblem (ProblemPtr_t problem);

      private:
        DevicePtr_t robot_;
        /// The pointer should point to the same object as core::Problem.
        ProblemPtr_t problem_;
        graph::GraphPtr_t constraintGraph_;

        GraspsMap_t graspsMap_;
    }; // class ProblemSolver
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_PROBLEM_SOLVER_HH
