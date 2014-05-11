// Copyright (c) 2014 CNRS
// Author: Florent Lamiraux
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
# include <hpp/core/problem-solver.hh>
# include <hpp/model/device.hh>
# include <hpp/manipulation/object.hh>
# include <hpp/manipulation/robot.hh>

namespace hpp {
  namespace manipulation {
    class HPP_MANIPULATION_DLLAPI ProblemSolver : public core::ProblemSolver
    {
    public:
      typedef core::ProblemSolver parent_t;
      typedef std::vector <std::string> Names_t;
      /// Destructor
      virtual ~ProblemSolver ()
      {
      }
      ProblemSolver () : core::ProblemSolver (), robot_ (),
	robotsAndObjects_ (), NumericalConstraintMap_ ()
	{
	}
      /// Set robot
      /// Check that robot is of type hpp::manipulation::Robot
      virtual void robot (const DevicePtr_t& robot)
      {
	robot_ = HPP_DYNAMIC_PTR_CAST (Robot, robot);
	assert (robot_);
	parent_t::robot (robot);
      }

      /// \name Robots and objects access
      /// \{

      /// Add a single robot before building a composite robot
      /// \param name key of the robot as stored in an internal map.
      /// \param robot robot that is stored.
      void addRobot (const std::string& name, const DevicePtr_t& robot)
      {
	robotsAndObjects_ [name] = robot;
      }

      /// Add an object before building a composite robot
      /// \param name key of the robot as stored in an internal map.
      /// \param object object that is stored.
      void addObject (const std::string& name, const ObjectPtr_t& object)
      {
	robotsAndObjects_ [name] = object;
      }

      /// Get robot
      const RobotPtr_t& robot () const
      {
	return robot_;
      }

      /// Get robot with given name
      ///
      /// throw if no robot is registered with this name.
      DevicePtr_t robot (const std::string& name) const;
      
      /// Get object with given name
      ///
      /// throw if no object is registered with this name.
      ObjectPtr_t object (const std::string& name) const;
      /// \}

      /// \name Constraints
      /// \{

      /// Add a a numerical constraint in the map.
      void addNumericalConstraint (const std::string& name,
				   const NumericalConstraintPtr_t& constraint)
      {
	NumericalConstraintMap_ [name] = constraint;
      }

      /// Get constraint with given name
      NumericalConstraintPtr_t constraint (const std::string& name)
      {
	return NumericalConstraintMap_ [name];
      }
      /// \}

      /// Build a composite robot from several robots and objects
      /// \param robotName Name of the composite robot,
      /// \param robotNames Names of the robots stored internally that have been
      ///        added by method addRobot.
      ///
      /// Objects are detected by dynamic cast.
      void buildCompositeRobot (const std::string& robotName,
				const Names_t& robotNames);
    private:
      typedef std::map <const std::string, DevicePtr_t> RobotsandObjects_t;
      RobotPtr_t robot_;
      /// Map of single robots to store before building a composite robot.
      RobotsandObjects_t robotsAndObjects_;
      /// Map of constraints
      NumericalConstraintMap_t NumericalConstraintMap_;
    }; // class ProblemSolver
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_PROBLEM_SOLVER_HH
