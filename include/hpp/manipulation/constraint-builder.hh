///
/// Copyright (c) 2014 CNRS
/// Authors: Florent Lamiraux
///
///
// This file is part of hpp-manipulation.
// hpp-manipulation is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-manipulation is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-manipulation. If not, see
// <http://www.gnu.org/licenses/>.

#ifndef HPP_MANIPULATION_CONSTRAINT_BUILDER_HH
# define HPP_MANIPULATION_CONSTRAINT_BUILDER_HH

# include <hpp/model/device.hh>
# include <hpp/constraints/relative-transformation.hh>
# include <hpp/manipulation/grasp.hh>
# include <hpp/manipulation/object.hh>
# include <hpp/manipulation/robot.hh>

namespace hpp {
  namespace manipulation {
    /// Constraint builder
    ///
    /// This class takes as input a manipulation robot (class Robot) and
    /// an instance of Grasp. It computes a constraint as a
    /// \link DifferentiableFunction \endlink that keeps the relative position between
    /// the robot and the object handle.
    class HPP_MANIPULATION_DLLAPI ConstraintBuilder {
    public:
      ConstraintBuilder ()
	{
	}
      DifferentiableFunctionPtr_t operator () (const RobotPtr_t& robot,
					       const GraspPtr_t& grasp) const
      {
	return RelativeTransformation::create
	  (robot, robot->joint (grasp->joint ()), grasp->handle ().joint,
	   inverse (grasp->handle ().localPosition) *
	   grasp->handlePositionInJoint ());
      }
    }; // class ConstraintBuilder
  } // namespace manipulation
} // namespace hpp
#endif // HPP_MANIPULATION_CONSTRAINT_BUILDER_HH
