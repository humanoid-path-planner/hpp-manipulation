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

#include <boost/assign/list_of.hpp>
#include <fcl/math/transform.h>
#include <hpp/model/joint.hh>
#include <hpp/constraints/relative-position.hh>
#include <hpp/constraints/relative-transformation.hh>
#include <hpp/manipulation/handle.hh>
#include <hpp/model/gripper.hh>
#include <hpp/manipulation/robot.hh>

namespace hpp {
  namespace manipulation {

    DifferentiableFunctionPtr_t Handle::createGrasp
    (const GripperPtr_t& gripper) const
    {
      using boost::assign::list_of;
      std::vector <bool> mask = list_of (true)(true)(true)(true)(true)(true);
      return RelativeTransformation::create
	(gripper->joint()->robot(), gripper->joint (), joint(), inverse (localPosition()) *
	 gripper->objectPositionInJoint (), mask);
    }

    DifferentiableFunctionPtr_t Handle::createPreGrasp
    (const GripperPtr_t& gripper) const
    {
      using boost::assign::list_of;
      std::vector <bool> mask = list_of (false)(true)(true)(true)(true)(true);
      Transform3f transform = inverse (localPosition()) * gripper->objectPositionInJoint ();
      return RelativeTransformation::create
	(gripper->joint()->robot(), gripper->joint (), joint(), transform, mask);
    }

    DifferentiableFunctionPtr_t Handle::createPreGraspOnLine
    (const GripperPtr_t& gripper, const value_type& shift) const
    {
      using boost::assign::list_of;
      std::vector <bool> mask = list_of (true)(false)(false);
      Transform3f transform = inverse (localPosition()) * gripper->objectPositionInJoint ();
      fcl::Vec3f target = transform.getTranslation () + fcl::Vec3f (shift,0,0);
      return RelativePosition::create
        (gripper->joint()->robot(), gripper->joint (), joint(), target, fcl::Vec3f (0,0,0), mask);
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
  } // namespace manipulation
} // namespace hpp
std::ostream& operator<< (std::ostream& os,
			  const hpp::manipulation::Handle& handle)
{
  return handle.print (os);
}
