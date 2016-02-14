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

#include <hpp/manipulation/handle.hh>

#include <hpp/util/debug.hh>
#include <hpp/model/device.hh>

#include <boost/assign/list_of.hpp>

#include <hpp/fcl/math/transform.h>

#include <hpp/model/joint.hh>
#include <hpp/model/gripper.hh>

#include <hpp/constraints/relative-position.hh>
#include <hpp/constraints/relative-transformation.hh>

#include <hpp/core/numerical-constraint.hh>
#include <hpp/core/explicit-numerical-constraint.hh>
#include <hpp/core/explicit-relative-transformation.hh>

namespace hpp {
  namespace manipulation {
    namespace {
      struct ZeroDiffFunc : public constraints::DifferentiableFunction {
        ZeroDiffFunc (size_type sIn, size_type sInD,
            std::string name=std::string("Empty function"))
          : DifferentiableFunction (sIn, sInD, 0, 0, name)
        {
          context ("Grasp complement");
        }

        inline void impl_compute (vectorOut_t, vectorIn_t) const {}
        inline void impl_jacobian (matrixOut_t, vectorIn_t) const {}
      };
    }

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

    NumericalConstraintPtr_t Handle::createGrasp
    (const GripperPtr_t& gripper) const
    {
      using boost::assign::list_of;
      using core::ExplicitRelativeTransformation;
      // If handle is on a freeflying object, create an explicit constraint
      if (isHandleOnR3SO3 (*this)) {
	return ExplicitRelativeTransformation::create
	  ("Explicit_relative_transform_" + name () + "_" + gripper->name (),
	   gripper->joint ()->robot (), gripper->joint (), joint (),
	   gripper->objectPositionInJoint (), localPosition());
      }
      std::vector <bool> mask = list_of (true)(true)(true)(true)(true)(true);
      return NumericalConstraintPtr_t
	(NumericalConstraint::create (RelativeTransformation::create
				      ("Transformation_(1,1,1,1,1,1)_" + name ()
				       + "_" + gripper->name (),
				       gripper->joint()->robot(),
				       gripper->joint (), joint (),
				       gripper->objectPositionInJoint (),
				       localPosition(), mask)));
    }

    NumericalConstraintPtr_t Handle::createGraspComplement
    (const GripperPtr_t& gripper) const
    {
      core::DevicePtr_t robot = gripper->joint()->robot();
      return NumericalConstraint::create (
          boost::shared_ptr <ZeroDiffFunc> (new ZeroDiffFunc (
            robot->configSize(), robot->numberDof (),
            "Transformation_(0,0,0,0,0,0)_" + name () + "_" + gripper->name ()
            ))
          );
    }

    NumericalConstraintPtr_t Handle::createPreGrasp
    (const GripperPtr_t& gripper, const value_type& shift) const
    {
      using boost::assign::list_of;
      std::vector <bool> mask = list_of (true)(true)(true)(true)(true)(true);
      Transform3f transform = gripper->objectPositionInJoint ()
        * Transform3f (fcl::Vec3f (shift,0,0));
      return NumericalConstraintPtr_t
	(NumericalConstraint::create (RelativeTransformation::create
				      ("Transformation_(1,1,1,1,1,1)_" + name ()
				       + "_" + gripper->name (),
				       gripper->joint()->robot(),
				       gripper->joint (), joint (),
                                       transform, localPosition(), mask)));
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
