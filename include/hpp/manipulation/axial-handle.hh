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
# include <hpp/manipulation/deprecated.hh>

namespace hpp {
  namespace manipulation {
    /// Handle symmetric around its z-axis. The constraint defined for a grasp
    /// by a gripper is free to rotate around z-axis.
    /// \deprecated This class is deprecated. Use class Handle with appropriate
    ///             mask
    class HPP_MANIPULATION_DLLAPI AxialHandle : public Handle
    {
    public:
      static std::string className;
      /// Create constraint corresponding to a gripper grasping this object
      /// \param robot the robot that grasps the handle,
      /// \param grasp object containing the grasp information
      /// \return the constraint of relative position between the handle and
      ///         the gripper. The rotation around x is not constrained.
      /// \deprecated This class is deprecated. The method creates an instance
      ///             of Handle with appropriate mask
      static AxialHandlePtr_t create (const std::string& name,
				      const Transform3f& localPosition,
				      const JointPtr_t& joint)
        HPP_MANIPULATION_DEPRECATED;

      /// Return a pointer to the copy of this
      virtual HandlePtr_t clone () const;

      /// Create constraint corresponding to a gripper grasping this handle
      /// \param gripper object containing the gripper information
      /// \return the constraint of relative transformation between the handle
      ///         and the gripper.
      /// \note 5 degrees of freedom are constrained. Rotation around z is free.
      virtual NumericalConstraintPtr_t createGrasp
      (const GripperPtr_t& gripper, std::string name) const;

      /// Create complement constraint of gripper grasping this handle
      /// \param gripper object containing the gripper information
      /// \return trivial constraint
      /// \note 1 degree of freedom is constrained: rotation around z.
      /// \note the constraint is parameterizable: non constant right hand side.
      virtual NumericalConstraintPtr_t createGraspComplement
      (const GripperPtr_t& gripper, std::string name) const;

      /// Create constraint corresponding to a pregrasping task.
      /// \param gripper object containing the gripper information
      /// \return the constraint of relative transformation between the handle and
      ///         the gripper.
      /// \note The translation along x-axis and the rotation around z-axis are not constrained.
      /// \todo this function is never called. It should follow changes of
      ///       Handle::createPreGrasp prototype.
      virtual NumericalConstraintPtr_t createPreGrasp
      (const GripperPtr_t& gripper, const value_type& shift, std::string name) const;

      virtual std::ostream& print (std::ostream& os) const;
    protected:
      /// Constructor
      /// \param robot the robot that grasps the handle,
      /// \param grasp object containing the grasp information
      /// \return the constraint of relative position between the handle and
      ///         the gripper. The rotation around x is not constrained.
      AxialHandle (const std::string& name, const Transform3f& localPosition,
		   const JointPtr_t& joint)  HPP_MANIPULATION_DEPRECATED;

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
