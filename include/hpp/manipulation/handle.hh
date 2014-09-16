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

#ifndef HPP_MANIPULATION_HANDLE_HH
# define HPP_MANIPULATION_HANDLE_HH

# include <fcl/math/transform.h>
# include <hpp/manipulation/config.hh>
# include <hpp/manipulation/fwd.hh>

namespace hpp {
  namespace manipulation {
    /// Part of an object that is aimed at being grasped
    class Handle
    {
    public:
      /// Create constraint corresponding to a gripper grasping this object
      /// \param robot the robot that grasps the handle,
      /// \param grasp object containing the grasp information
      /// \return the constraint of relative position between the handle and
      ///         the gripper.
      static HandlePtr_t create (const std::string& name,
				 const Transform3f& localPosition,
				 const JointPtr_t& joint)
      {
	Handle* ptr = new Handle (name, localPosition, joint);
	HandlePtr_t shPtr (ptr);
	ptr->init (shPtr);
	return shPtr;
      }
      /// Return a pointer to the copy of this
      virtual HandlePtr_t clone () const;

      /// \name Name
      /// \{

      /// Get name
      const std::string& name () const
      {
	return name_;
      }
      /// Set name
      void name (const std::string& n)
      {
	name_ = n;
      }
      /// \}

      /// \name Joint
      /// \{

      /// Get joint to which the handle is linked
      const JointPtr_t& joint () const
      {
	return joint_;
      }
      /// Set joint to which the handle is linked
      void joint (const JointPtr_t& joint)
      {
	joint_ = joint;
      }
      /// \}

      /// Get local position in joint frame
      const Transform3f& localPosition () const
      {
	return localPosition_;
      }

      /// Create constraint corresponding to a gripper grasping this object
      /// \param gripper object containing the gripper information
      /// \return the constraint of relative position between the handle and
      ///         the gripper.
      /// \note the constraint may constrain less than 6 degrees of freedom
      ///       if the handle can be grasped in different positions. In case of
      ///       a handle symmetric with respect to its x-axis for instance, the
      ///       rotation around x is not constrained.
      virtual DifferentiableFunctionPtr_t createGrasp
      (const GripperPtr_t& gripper) const;

      /// Create constraint corresponding to a gripper grasping this object
      /// \param gripper object containing the gripper information
      /// \return the constraint of relative position between the handle and
      ///         the gripper.
      /// \note the constraint may constrain less than 6 degrees of freedom
      ///       if the handle can be grasped in different positions. In case of
      ///       a handle symmetric with respect to its x-axis for instance, the
      ///       rotation around x is not constrained.
      virtual DifferentiableFunctionPtr_t createPreGrasp
      (const GripperPtr_t& gripper) const;

      /// Create a constraint define on the line of approach of the object
      /// This is useful for inequality constraints.
      /// \name gripper The hpp::model::Gripper
      /// \name shift a position on the line wrt the axis of the handle joint.
      /// \return A hpp::constraints::RelativePosition constraint.
      virtual DifferentiableFunctionPtr_t createPreGraspOnLine
        (const GripperPtr_t& gripper, const value_type& shift) const;

      static DifferentiableFunctionPtr_t createGrasp
      (const GripperPtr_t& gripper,const HandlePtr_t& handle)
      {
        return handle->createGrasp(gripper);
      }

      virtual std::ostream& print (std::ostream& os) const;

    protected:
      /// Constructor
      /// \param robot the robot that grasps the handle,
      /// \param grasp object containing the grasp information
      /// \return the constraint of relative position between the handle and
      ///         the gripper.
      Handle (const std::string& name, const Transform3f& localPosition,
	      const JointPtr_t& joint) : name_ (name),
					 localPosition_ (localPosition),
					 joint_ (joint), weakPtr_ ()
      {
      }
      void init (HandleWkPtr_t weakPtr)
      {
	weakPtr_ = weakPtr;
      }
    private:
      std::string name_;
      /// Position of the handle in the joint frame.
      Transform3f localPosition_;
      /// Joint to which the handle is linked.
      JointPtr_t joint_;
      /// Weak pointer to itself
      HandleWkPtr_t weakPtr_;
    }; // class Handle
  } // namespace manipulation
} // namespace hpp
std::ostream& operator<< (std::ostream& os,
			  const hpp::manipulation::Handle& handle);

#endif // HPP_MANIPULATION_HANDLE_HH
