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

#ifndef HPP_MANIPULATION_GRASP_HH
# define HPP_MANIPULATION_GRASP_HH

# include <hpp/manipulation/handle.hh>

namespace hpp {
  namespace manipulation {
    /// Constraint between the position of a robot joint and of an object handle
    class HPP_MANIPULATION_DLLAPI Grasp
    {
    public:
      /// Return a shared pointer to new instance
      /// \param joint joint of the robot that holds the handle,
      /// \param handle handle that is grasped,
      /// \param handlePositionInJoint handle position in the the grasping
      ///        joint.
      static GraspPtr_t create (const JointPtr_t& joint,
				const HandlePtr_t& handle,
				const Transform3f& handlePositionInJoint)
      {
	Grasp* ptr = new Grasp (joint, handle, handlePositionInJoint);
	return GraspPtr_t (ptr);
      }
      /// Get handle that is grasped
      const HandlePtr_t& handle () const
      {
	return handle_;
      }
      /// Get joint that grasps
      const JointPtr_t& joint () const
      {
	return joint_;
      }
      /// Get handle position in the the grasping joint
      const Transform3f& handlePositionInJoint () const
      {
	return handlePositionInJoint_;
      }
    protected:
      /// Constructor
      /// \param joint joint of the robot that holds the handle,
      /// \param handle handle that is grasped,
      /// \param handlePositionInJoint handle position in the the grasping
      ///        joint.
      Grasp (const JointPtr_t& joint, const HandlePtr_t& handle,
	     const Transform3f& handlePositionInJoint) :
	joint_ (joint), handle_ (handle),
	handlePositionInJoint_ (handlePositionInJoint)
      {
      }
    private:
      /// Joint of the robot that holds the handle.
      JointPtr_t joint_;
      HandlePtr_t handle_;
      Transform3f handlePositionInJoint_;
    }; // class Grasp
  } // namespace manipulation
} // namespace hpp
#endif // HPP_MANIPULATION_GRASP_HH
