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

#include <hpp/manipulation/axial-handle.hh>

#include <boost/assign/list_of.hpp>

#include <hpp/fcl/math/transform.h>

#include <hpp/model/joint.hh>
#include <hpp/model/gripper.hh>

#include <hpp/constraints/generic-transformation.hh>

#include <hpp/core/numerical-constraint.hh>

namespace hpp {
  namespace manipulation {

    NumericalConstraintPtr_t AxialHandle::createGrasp
    (const GripperPtr_t& gripper) const
    {
      using boost::assign::list_of;
      std::vector <bool> mask = list_of (true)(true)(true)(true)(true)(false);
      return NumericalConstraintPtr_t
	(NumericalConstraint::create (RelativeTransformation::create
				      ("Transformation_(1,1,1,1,1,0)_" + name ()
				       + "_" + gripper->name (),
				       gripper->joint()->robot(),
				       gripper->joint (), joint (),
				       gripper->objectPositionInJoint (),
				       localPosition(), mask)));
    }

    NumericalConstraintPtr_t AxialHandle::createGraspComplement
    (const GripperPtr_t& gripper) const
    {
      using boost::assign::list_of;
      std::vector <bool> mask = list_of (false)(false)(false)(false)(false)
        (true);
      return NumericalConstraintPtr_t
	(NumericalConstraint::create (RelativeTransformation::create
				      ("Transformation_(0,0,0,0,0,1)_" + name ()
				       + "_" + gripper->name (),
				       gripper->joint()->robot(),
				       gripper->joint (), joint (),
				       gripper->objectPositionInJoint (),
				       localPosition(), mask)));
    }

    NumericalConstraintPtr_t AxialHandle::createPreGrasp
    (const GripperPtr_t& gripper, const value_type& shift) const
    {
      using boost::assign::list_of;
      std::vector <bool> mask = list_of (true)(true)(true)(true)(true)(false);
      Transform3f transform = gripper->objectPositionInJoint ()
        * Transform3f (fcl::Vec3f (shift,0,0));
      return NumericalConstraintPtr_t
	(NumericalConstraint::create (RelativeTransformation::create
				      ("Transformation_(1,1,1,1,1,0)_" + name ()
				       + "_" + gripper->name (),
				       gripper->joint()->robot(),
				       gripper->joint (), joint (),
				       transform,
				       localPosition(), mask)));
    }

    NumericalConstraintPtr_t AxialHandle::createPreGraspComplement
    (const GripperPtr_t& gripper, const value_type& shift,
     const value_type& width) const
    {
      using boost::assign::list_of;
      using core::DoubleInequality;
      std::vector <bool> mask = list_of (true)(false)(false)(false)(false)
        (false);
      Transform3f transform = gripper->objectPositionInJoint ()
        * Transform3f (fcl::Vec3f (shift,0,0));
      return NumericalConstraintPtr_t
	(NumericalConstraint::create (RelativeTransformation::create
				      ("Transformation_(1,0,0,0,0,0)_" + name ()
				       + "_" + gripper->name (),
				       gripper->joint()->robot(),
				       gripper->joint (), joint (),
				       transform, localPosition(), mask),
				      DoubleInequality::create (width)));
    }

    HandlePtr_t AxialHandle::clone () const
    {
      AxialHandlePtr_t self = weakPtr_.lock ();
      return AxialHandle::create (self->name (), self->localPosition (),
				  self->joint ());
    }

    std::ostream& AxialHandle::print (std::ostream& os) const
    {
      os << "name :" << name () << " (axial)" << std::endl;
      os << "local position :" << localPosition () << std::endl;
      os << "joint :" << joint ()->name () << std::endl;
      return os;
    }
  } // namespace manipulation
} // namespace hpp
