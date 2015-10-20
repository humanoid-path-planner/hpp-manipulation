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

#include <hpp/util/debug.hh>
#include <hpp/model/device.hh>
#include <hpp/core/explicit-numerical-constraint.hh>
#include <hpp/manipulation/handle.hh>

#include <boost/assign/list_of.hpp>

#include <hpp/fcl/math/transform.h>

#include <hpp/model/joint.hh>
#include <hpp/model/gripper.hh>

#include <hpp/constraints/relative-position.hh>
#include <hpp/constraints/relative-transformation.hh>

namespace hpp {
  namespace manipulation {

    using core::ExplicitNumericalConstraint;
    using constraints::DifferentiableFunction;
    using core::SizeInterval_t;
    using core::SizeIntervals_t;

    bool isHandleOnR3SO3 (const Handle& handle)
    {
      if ((dynamic_cast <model::JointSO3*> (handle.joint ())) &&
	  (dynamic_cast <model::JointTranslation <3>* >
	   (handle.joint ()->parentJoint ()))) {
	return true;
      }
      return false;
    }

    HPP_PREDEF_CLASS (ObjectPositionWrtRobot);
    typedef boost::shared_ptr <ObjectPositionWrtRobot>
    ObjectPositionWrtRobotPtr_t;
    class ObjectPositionWrtRobot : public DifferentiableFunction
    {
    public:
      typedef Eigen::Matrix <value_type, 3, Eigen::Dynamic> HalfJacobian_t;
      virtual ~ObjectPositionWrtRobot ()
      {
      }

      static ObjectPositionWrtRobotPtr_t create (const GripperPtr_t& gripper,
						 const Handle& handle,
						 const std::string& name)
      {
	return ObjectPositionWrtRobotPtr_t (new ObjectPositionWrtRobot
					    (gripper, handle, name));
      }
    protected:
      ObjectPositionWrtRobot (const GripperPtr_t& gripper,
			      const Handle& handle,
			      const std::string& name) :
	DifferentiableFunction (gripper->joint ()->robot ()->configSize () - 7,
				gripper->joint ()->robot ()->numberDof () - 6,
				7, 6, name),
	robot_ (gripper->joint ()->robot ()),
	objectJoint_ (handle.joint ()->parentJoint ()),
	gripperJoint_ (gripper->joint ()),
	robotConfigVar_ (),
	robotDofVar_ (), objectInGripperJoint_ (), configuration_ ()
      {
	configuration_.resize (robot_->configSize ());
	// Extract configuration parameters relative to robot without object
	if (objectJoint_->rankInConfiguration () != 0) {
	  robotConfigVar_.push_back (SizeInterval_t
				     (0, objectJoint_->rankInConfiguration ()));
	}
	if (robot_->configSize () != objectJoint_->rankInConfiguration () + 7) {
	  robotConfigVar_.push_back (SizeInterval_t
				     (objectJoint_->rankInConfiguration () + 7,
				      robot_->configSize () -
				      objectJoint_->rankInConfiguration ()
				      - 7));
	}
	if (objectJoint_->rankInVelocity () != 0) {
	  robotDofVar_.push_back (SizeInterval_t
				  (0, objectJoint_->rankInVelocity ()));
	}
	if (robot_->numberDof () != objectJoint_->rankInVelocity () + 6) {
	  robotDofVar_.push_back (SizeInterval_t
				  (objectJoint_->rankInVelocity () + 6,
				   robot_->numberDof () -
				   objectJoint_->rankInVelocity () - 6));
	}
	// Compute position of object in gripper joint frame
	objectInGripperJoint_ = gripper->objectPositionInJoint () *
	  inverse (handle.localPosition ());

	GOcross_.setZero ();
	Jgj_v_.resize (3, robot_->numberDof () - 6); Jgj_v_.setZero ();
	Jgj_omega_.resize (3, robot_->numberDof () - 6); Jgj_omega_.setZero ();
      }

      void computeRobotForwardKinematics (vectorIn_t argument) const
      {
	// Be sure to initialize all configuration variables
	configuration_ = robot_->neutralConfiguration ();
	size_type index = 0;
	for (SizeIntervals_t::const_iterator it = robotConfigVar_.begin ();
	     it != robotConfigVar_.end (); ++it) {
	  configuration_.segment (it->first, it->second) =
	    argument.segment (index, it->second);
	  index += it->second;
	}
	robot_->currentConfiguration (configuration_);
	robot_->computeForwardKinematics ();
      }

      virtual void impl_compute (vectorOut_t result, vectorIn_t argument) const
      {
	computeRobotForwardKinematics (argument);
	objectPosition_ = gripperJoint_->currentTransformation () *
	  objectInGripperJoint_;
	result [0] = objectPosition_.getTranslation () [0];
	result [1] = objectPosition_.getTranslation () [1];
	result [2] = objectPosition_.getTranslation () [2];
	result [3] = objectPosition_.getQuatRotation ().getW ();
	result [4] = objectPosition_.getQuatRotation ().getX ();
	result [5] = objectPosition_.getQuatRotation ().getY ();
	result [6] = objectPosition_.getQuatRotation ().getZ ();
      }

      virtual void impl_jacobian (matrixOut_t jacobian, vectorIn_t arg) const
      {
	computeRobotForwardKinematics (arg);
	vector3_t GO (gripperJoint_->currentTransformation ().transform
		      (objectInGripperJoint_.getTranslation ()));
	GOcross_ (0,1) = -GO [2]; GOcross_ (1,0) = GO [2];
	GOcross_ (0,2) = GO [1]; GOcross_ (2,0) = -GO [1];
	GOcross_ (1,2) = -GO [0]; GOcross_ (2,1) = GO [0];
	// Fill Jacobian without columns relative to object position
	size_type index = 0;
	for (SizeIntervals_t::const_iterator it = robotDofVar_.begin ();
	     it != robotDofVar_.end (); ++it) {
	  Jgj_v_.block (0, index, 3, it->second) =
	    gripperJoint_->jacobian (). block (0, it->first, 3, it->second);
	  Jgj_omega_.block (0, index, 3, it->second) =
	    gripperJoint_->jacobian (). block (3, it->first, 3, it->second);
	  index += it->second;
	}
	jacobian.topRows <3> () = Jgj_v_ - GOcross_ * Jgj_omega_;
	jacobian.bottomRows <3> () = Jgj_omega_;
      }

    private:
      model::DevicePtr_t robot_;
      // First joint of the freeflying object, should be a TranslationJoint <3>
      JointPtr_t objectJoint_;
      // Joint that holds the gripper
      JointPtr_t gripperJoint_;
      // Set of configuration variables corresponding to composite robot minus
      // object.
      SizeIntervals_t robotConfigVar_;
      // Set of degrees of freedom corresponding to composite robot minus
      // object.
      SizeIntervals_t robotDofVar_;
      // Position of object in gripper joint frame
      Transform3f objectInGripperJoint_;
      // Configuration of the composite robot
      mutable Configuration_t configuration_;
      // Object position
      mutable Transform3f objectPosition_;
      // Jacobian of gripper joint without columns relative to object
      mutable HalfJacobian_t Jgj_v_;
      mutable HalfJacobian_t Jgj_omega_;
      mutable Eigen::Matrix<value_type, 3, 3> GOcross_;
    }; // class ObjectPositionWrtRobot

    DifferentiableFunctionPtr_t Handle::createGrasp
    (const GripperPtr_t& gripper) const
    {
      using boost::assign::list_of;
      // If handle is on a freeflying object, create an explicit constraint
      if (isHandleOnR3SO3 (*this)) {
	JointPtr_t objectJoint (joint ()->parentJoint ());
	SizeIntervals_t outputConf, outputVelocity;
	outputConf.push_back (SizeInterval_t
			      (objectJoint->rankInConfiguration (), 7));
	outputVelocity.push_back (SizeInterval_t
				  (objectJoint->rankInVelocity (), 6));
	DifferentiableFunctionPtr_t objectPositionWrtRobotConf
	  (ObjectPositionWrtRobot::create (gripper, *this,
					   "Explicit transformation_" + name ()
					   + "_" + gripper->name ()));
	return core::ExplicitNumericalConstraint::create
	  (joint ()->robot (), objectPositionWrtRobotConf, outputConf,
	   outputVelocity);
      }
      std::vector <bool> mask = list_of (true)(true)(true)(true)(true)(true);
      return RelativeTransformation::create
	("Transformation_(1,1,1,1,1,1)_" + name () + "_" + gripper->name (),
	 gripper->joint()->robot(), gripper->joint (), joint (),
	 gripper->objectPositionInJoint (), localPosition(), mask);
    }

    DifferentiableFunctionPtr_t Handle::createGraspComplement
    (const GripperPtr_t& gripper) const
    {
      using boost::assign::list_of;
      std::vector <bool> mask = list_of (false)(false)(false)(false)(false)
	(false);
      return RelativeTransformation::create
	("Transformation_(0,0,0,0,0,0)_" + name () + "_" + gripper->name (),
	 gripper->joint()->robot(), gripper->joint (), joint (),
	 gripper->objectPositionInJoint (), localPosition(), mask);
    }

    DifferentiableFunctionPtr_t Handle::createPreGrasp
    (const GripperPtr_t& gripper) const
    {
      using boost::assign::list_of;
      std::vector <bool> mask = list_of (false)(true)(true)(true)(true)(true);
      return RelativeTransformation::create
	("Transformation_(0,1,1,1,1,1)_" + name () + "_" + gripper->name (),
	 gripper->joint()->robot(), gripper->joint (), joint (),
	 gripper->objectPositionInJoint (), localPosition(), mask);
    }

    DifferentiableFunctionPtr_t Handle::createPreGraspComplement
    (const GripperPtr_t& gripper, const value_type& shift) const
    {
      using boost::assign::list_of;
      std::vector <bool> mask = list_of (true)(false)(false)(false)(false)
	(false);
      Transform3f transform (gripper->objectPositionInJoint ().getRotation (),
			     gripper->objectPositionInJoint ().getTranslation ()
			     + fcl::Vec3f (shift,0,0));
      return RelativeTransformation::create
	("Transformation_(1,0,0,0,0,0)_" + name () + "_" + gripper->name (),
	 gripper->joint()->robot(), gripper->joint (), joint (),
	 transform, localPosition(), mask);
    }

    HandlePtr_t Handle::clone () const
    {
      HandlePtr_t self = weakPtr_.lock ();
      return Handle::create (self->name (), self->localPosition (),
          self->joint ());
    }

    std::ostream& Handle::print (std::ostream& os) const
    {
      os << "name :" << name () << std::endl;
      os << "local position :" << localPosition () << std::endl;
      os << "joint :" << joint ()->name () << std::endl;
      return os;
    }

    std::ostream& operator<< (std::ostream& os, const Handle& handle)
    {
      return handle.print (os);
    }
  } // namespace manipulation
} // namespace hpp
