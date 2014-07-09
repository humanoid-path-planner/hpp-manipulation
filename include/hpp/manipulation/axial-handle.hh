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

#ifndef HPP_MANIPULATION_AXIAL_HANDLE_HH
# define HPP_MANIPULATION_AXIAL_HANDLE_HH

# include <hpp/manipulation/handle.hh>

namespace hpp {
  namespace manipulation {
    /// Handle symmetric around its x-axis.
    class AxialHandle : public Handle
    {
    public:
      /// Create constraint corresponding to a gripper grasping this object
      /// \param robot the robot that grasps the handle,
      /// \param grasp object containing the grasp information
      /// \return the constraint of relative position between the handle and
      ///         the gripper. The rotation around x is not constrained.
      static AxialHandlePtr_t create (const std::string& name,
				      const Transform3f& localPosition,
				      const JointPtr_t& joint)
      {
	AxialHandle* ptr = new AxialHandle (name, localPosition, joint);
	AxialHandlePtr_t shPtr (ptr);
	ptr->init (shPtr);
	return shPtr;
      }
      /// Return a pointer to the copy of this
      virtual HandlePtr_t clone () const;

      /// Create constraint corresponding to a gripper grasping this object
      /// \param robot the robot that grasps the handle,
      /// \param grasp object containing the grasp information
      /// \return the constraint of relative position between the handle and
      ///         the gripper. The rotation around x is not constrained.
      virtual DifferentiableFunctionPtr_t createGrasp
      (const GripperPtr_t& gripper) const;

      virtual std::ostream& print (std::ostream& os) const;
    protected:
      /// Constructor
      /// \param robot the robot that grasps the handle,
      /// \param grasp object containing the grasp information
      /// \return the constraint of relative position between the handle and
      ///         the gripper. The rotation around x is not constrained.
      AxialHandle (const std::string& name, const Transform3f& localPosition,
		   const JointPtr_t& joint) : Handle (name, localPosition,
						      joint), weakPtr_ ()
      {
      }
      void init (const AxialHandleWkPtr_t& weakPtr)
      {
	Handle::init (weakPtr);
	weakPtr_ = weakPtr;
      }
      /// Weak pointer to itself
      AxialHandleWkPtr_t weakPtr_;
    }; // class AxialHandle
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_AXIAL_HANDLE_HH