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

#ifndef HPP_MANIPULATION_GRIPPER_HH
# define HPP_MANIPULATION_GRIPPER_HH

# include <hpp/manipulation/handle.hh>

namespace hpp {
  namespace manipulation {
    /// Constraint between the position of a robot joint and of an object handle
    class HPP_MANIPULATION_DLLAPI Gripper
    {
    public:
      /// Return a shared pointer to new instance
      /// \param joint joint of the robot that will hold handles,
      /// \param handlePositionInJoint handle position in the the grasping
      ///        joint.
      static GripperPtr_t create (const std::string& name, const JointPtr_t& joint,
				const Transform3f& handlePositionInJoint)
      {
	Gripper* ptr = new Gripper (name, joint, handlePositionInJoint);
	return GripperPtr_t (ptr);
      }

      /// Get joint that Grippers
      const JointPtr_t& joint () const
      {
	return joint_;
      }
      /// Get handle position in the the Grippering joint
      const Transform3f& handlePositionInJoint () const
      {
	return handlePositionInJoint_;
      }
      ///get name
      const std::string& name () const
      {
	return name_;
      }
      /// Set name
      void name (const std::string& n)
      {
	name_ = n;
      }

      
      DifferentiableFunctionPtr_t createGrasp(HandlePtr_t& handle)
      {
        return handle->createGrasp(GripperPtr_t(this));
      }

    protected:
      /// Constructor
      /// \param joint joint of the robot that holds the handle,
      /// \param handlePositionInJoint handle position in the the grasping
      ///        joint.
      Gripper (const std::string& name, const JointPtr_t& joint,
	     const Transform3f& handlePositionInJoint) :
	joint_ (joint),
	handlePositionInJoint_ (handlePositionInJoint)
      {
      }
    private:
      /// Joint of the robot that holds handles.
      std::string name_;
      JointPtr_t joint_;
      Transform3f handlePositionInJoint_;
    }; // class Gripper
  } // namespace manipulation
} // namespace hpp
#endif // HPP_MANIPULATION_GRIPPER_HH
