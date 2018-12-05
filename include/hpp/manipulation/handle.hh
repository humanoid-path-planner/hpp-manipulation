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

# include <pinocchio/spatial/se3.hpp>

# include <hpp/manipulation/fwd.hh>
# include <hpp/manipulation/config.hh>

namespace hpp {
  namespace manipulation {
    typedef constraints::ImplicitPtr_t ImplicitPtr_t;
    /// Part of an object that is aimed at being grasped
    class HPP_MANIPULATION_DLLAPI Handle
    {
    public:
      static std::string className;
      virtual ~Handle () {};

      /// Create constraint corresponding to a gripper grasping this object
      /// \param robot the robot that grasps the handle,
      /// \param grasp object containing the grasp information
      /// \return the constraint of relative position between the handle and
      ///         the gripper.
      static HandlePtr_t create (const std::string& name,
				 const Transform3f& localPosition,
				 const DeviceWkPtr_t& robot,
				 const JointPtr_t& joint)
      {
	Handle* ptr = new Handle (name, localPosition, robot, joint);
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

      DevicePtr_t robot () const
      {
        return robot_.lock();
      }
      /// \}

      /// Get local position in joint frame
      const Transform3f& localPosition () const
      {
	return localPosition_;
      }

      /// Set constraint mask
      void mask (const std::vector<bool>& mask);

      /// Get constraint mask
      /// See mask(const std::vector<bool>&)
      const std::vector<bool>& mask () const
      { return mask_; }

      /// Create constraint corresponding to a gripper grasping this handle
      /// \param gripper object containing the gripper information
      /// \return the constraint of relative transformation between the handle
      ///         and the gripper.
      /// The degrees of freedom of the relative transformation that are
      /// constrained are determined by the mask.
      /// \sa constraints::Implicit::mask.
      /// The constraint is not parameterizable (has constant right hand side).
      virtual ImplicitPtr_t createGrasp
      (const GripperPtr_t& gripper, std::string name) const;

      /// Create complement constraint of gripper grasping this handle
      /// \param gripper object containing the gripper information
      /// \return complement constraint: constraint combined with its complement
      ///         constitute a full relative transformation constraint.
      /// The complement constraint is parameterizable (has non constant right
      /// hand side).
      virtual ImplicitPtr_t createGraspComplement
      (const GripperPtr_t& gripper, std::string name) const;

      /// Create constraint composed of grasp constraint and its complement
      /// \param gripper object containing the gripper information
      /// \return the composition of grasp constraint and its complement, that
      ///         that is a full relative transformation constraint.
      virtual ImplicitPtr_t createGraspAndComplement
      (const GripperPtr_t& gripper, std::string name) const;

      /// Create constraint corresponding to a pregrasping task.
      /// \param gripper object containing the gripper information
      /// \return the constraint of relative transformation between the handle and
      ///         the gripper.
      /// \note 6 DOFs of the relative transformation between the handle and the gripper
      ///       are constrained. The transformation is shifted along x-axis of
      ///       value shift.
      virtual ImplicitPtr_t createPreGrasp
      (const GripperPtr_t& gripper, const value_type& shift, std::string name) const;

      /// Get the clearance
      ///
      /// The clearance is a distance, from the center of the gripper and along
      /// the x-aixs, that "ensures" an object being at that distance is not
      /// colliding with this gripper.
      /// It also gives an order of magnitude of the size of the gripper.
      value_type clearance () const
      {
        return clearance_;
      }

      /// Set the clearance
      /// \sa clearance()
      void clearance (const value_type& clearance)
      {
        clearance_ = clearance;
      }

    protected:
      /// Constructor
      /// \param robot the robot that grasps the handle,
      /// \param grasp object containing the grasp information
      /// \return the constraint of relative position between the handle and
      ///         the gripper.
      Handle (const std::string& name, const Transform3f& localPosition,
              const DeviceWkPtr_t& robot, const JointPtr_t& joint) :
        name_ (name), localPosition_ (localPosition), joint_ (joint),
        robot_ (robot), clearance_ (0), mask_ (6, true), weakPtr_ ()
      {
      }
      void init (HandleWkPtr_t weakPtr)
      {
	weakPtr_ = weakPtr;
      }

      virtual std::ostream& print (std::ostream& os) const;

    private:
      std::string name_;
      /// Position of the handle in the joint frame.
      Transform3f localPosition_;
      /// Joint to which the handle is linked.
      JointPtr_t joint_;
      /// Pointer to the robot
      DeviceWkPtr_t robot_;
      /// Clearance
      value_type clearance_;
      /// Mask
      std::vector<bool> mask_;
      /// Weak pointer to itself
      HandleWkPtr_t weakPtr_;

      friend std::ostream& operator<< (std::ostream&, const Handle&);
    }; // class Handle

    std::ostream& operator<< (std::ostream& os, const Handle& handle);
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_HANDLE_HH
