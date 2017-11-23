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

#include <boost/assign/list_of.hpp>

#include <pinocchio/multibody/joint/joint.hpp>

#include <hpp/util/debug.hh>

#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/gripper.hh>

#include <hpp/constraints/generic-transformation.hh>

#include <hpp/core/numerical-constraint.hh>
#include <hpp/core/explicit-numerical-constraint.hh>
#include <hpp/core/explicit-relative-transformation.hh>

namespace hpp {
  namespace manipulation {
    namespace {
      static const matrix3_t I3 = matrix3_t::Identity();

      struct ZeroDiffFunc : public constraints::DifferentiableFunction {
        ZeroDiffFunc (size_type sIn, size_type sInD,
            std::string name=std::string("Empty function"))
          : DifferentiableFunction (sIn, sInD, LiegroupSpace::empty (), name)
        {
          context ("Grasp complement");
        }

        inline void impl_compute (LiegroupElement&, vectorIn_t) const {}
        inline void impl_jacobian (matrixOut_t, vectorIn_t) const {}
      };
    }

    using core::ExplicitNumericalConstraint;
    using constraints::DifferentiableFunction;

    bool isHandleOnFreeflyer (const Handle& handle)
    {
      if (handle.joint()->jointModel().shortname() == se3::JointModelFreeFlyer::classname()
          && !handle.joint ()->parentJoint ()) {
	return true;
      }
      return false;
    }

    inline bool is6Dmask (const std::vector<bool>& mask)
    {
      for (std::size_t i = 0; i < 6; ++i) if (!mask[i]) return false;
      return true;
    }

    inline std::vector<bool> complementMask (const std::vector<bool>& mask)
    {
      std::vector<bool> m(6);
      for (std::size_t i = 0; i < 6; ++i) m[i] = !mask[i];
      return m;
    }

    inline std::string maskToStr (const std::vector<bool>& mask)
    {
      std::stringstream ss;
      ss << "(";
      for (std::size_t i = 0; i < 5; ++i) ss << mask[i] << ",";
      ss << mask[5] << ")";
      return ss.str();
    }

    void Handle::mask (const std::vector<bool>& mask)
    {
      assert(mask.size() == 6);
      std::size_t nRotFree = 3;
      for (std::size_t i = 3; i < 6; ++i)
        if (mask[i]) nRotFree--;
      switch (nRotFree) {
        case 1: // TODO we should check the axis are properly aligned.
          break;
        case 2: // This does not make sense.
          throw std::logic_error("It is not allowed to constrain only one rotation");
          break;
      } 
      mask_ = mask;
    }

    NumericalConstraintPtr_t Handle::createGrasp
    (const GripperPtr_t& gripper, std::string n) const
    {
      using core::ExplicitRelativeTransformation;
      // If handle is on a freeflying object, create an explicit constraint
      if (is6Dmask(mask_) && isHandleOnFreeflyer (*this)) {
        if (n.empty())
          n = "Explicit_relative_transform_" + name() + "_" + gripper->name();
	return ExplicitRelativeTransformation::create
	  (n, gripper->joint ()->robot (), gripper->joint (), joint (),
	   gripper->objectPositionInJoint (), localPosition())->createNumericalConstraint();
      }
      if (n.empty())
        n = "Grasp_" + maskToStr(mask_) + "_" + name() + "_" + gripper->name();
      return NumericalConstraintPtr_t
	(NumericalConstraint::create (RelativeTransformation::create
				      (n,
				       gripper->joint()->robot(),
				       gripper->joint (), joint (),
				       gripper->objectPositionInJoint (),
				       localPosition(), mask_)));
    }

    NumericalConstraintPtr_t Handle::createGraspComplement
    (const GripperPtr_t& gripper, std::string n) const
    {
      core::DevicePtr_t robot = gripper->joint()->robot();
      if (is6Dmask(mask_)) {
        if (n.empty())
          n = "GraspComp_(0,0,0,0,0,0)_" + name () + "_" + gripper->name ();
        return NumericalConstraint::create (
            boost::shared_ptr <ZeroDiffFunc> (new ZeroDiffFunc (
                robot->configSize(), robot->numberDof (), n))
            );
      } else {
        // TODO handle cases where rotations or translation are allowed.
        std::vector<bool> Cmask = complementMask(mask_);
        if (n.empty())
          n = "Transformation_" + maskToStr(Cmask) + "_" + name ()
            + "_" + gripper->name ();
        RelativeTransformationPtr_t function = RelativeTransformation::create
          (n,
           gripper->joint()->robot(),
           gripper->joint (), joint (),
           gripper->objectPositionInJoint (),
           localPosition(), Cmask);
        return NumericalConstraintPtr_t (NumericalConstraint::create
            (function,
             core::ComparisonTypes::create(function->outputSize(),
                                           core::ComparisonType::Equality)
             ));
      }
    }

    NumericalConstraintPtr_t Handle::createPreGrasp
    (const GripperPtr_t& gripper, const value_type& shift, std::string n) const
    {
      using boost::assign::list_of;
      Transform3f transform = gripper->objectPositionInJoint ()
        * Transform3f (I3, vector3_t (shift,0,0));
      if (n.empty())
        n = "Pregrasp_ " + maskToStr(mask_) + "_" + name ()
          + "_" + gripper->name ();
      return NumericalConstraintPtr_t
	(NumericalConstraint::create (RelativeTransformation::create
				      (n,
				       gripper->joint()->robot(),
				       gripper->joint (), joint (),
                                       transform, localPosition(), mask_)));
    }

    HandlePtr_t Handle::clone () const
    {
      HandlePtr_t other = Handle::create (name (), localPosition (), joint ());
      other->mask(mask_);
      other->clearance(clearance_);
      return other;
    }

    std::ostream& Handle::print (std::ostream& os) const
    {
      os << "name :" << name () << std::endl;
      os << "local position :" << localPosition () << std::endl;
      os << "joint :" << joint ()->name () << std::endl;
      os << "mask :" << maskToStr (mask()) << std::endl;
      return os;
    }

    std::ostream& operator<< (std::ostream& os, const Handle& handle)
    {
      return handle.print (os);
    }
  } // namespace manipulation
} // namespace hpp
