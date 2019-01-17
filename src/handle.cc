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

#include <pinocchio/multibody/joint/joint-generic.hpp>

#include <hpp/util/debug.hh>

#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/joint-collection.hh>
#include <hpp/pinocchio/gripper.hh>

#include <hpp/constraints/differentiable-function.hh>
#include <hpp/constraints/implicit.hh>
#include <hpp/constraints/explicit/relative-pose.hh>

#include <hpp/manipulation/device.hh>

namespace hpp {
  namespace manipulation {
    using constraints::Implicit;
    using constraints::ImplicitPtr_t;
    using constraints::Explicit;
    using constraints::ExplicitPtr_t;
    std::string Handle::className ("Handle");
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

    using constraints::Explicit;
    using constraints::DifferentiableFunction;

    bool isHandleOnFreeflyer (const Handle& handle)
    {
      const JointPtr_t& joint = handle.joint();
      if (   joint
          && !joint->parentJoint()
          && joint->jointModel().shortname() == ::pinocchio::JointModelFreeFlyer::classname()) {
	return true;
      }
      return false;
    }

    inline std::size_t maskSize (const std::vector<bool>& mask)
    {
      std::size_t res (0);
      for (std::size_t i = 0; i < 6; ++i) {
        if (mask[i]) ++res;
      }
      return res;
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

    ImplicitPtr_t Handle::createGrasp
    (const GripperPtr_t& gripper, std::string n) const
    {
      if (n.empty()) {
        n = gripper->name() + "_grasps_" + name() + "_" + maskToStr (mask_);
      }
      // If handle is on a freeflying object, create an explicit constraint
      if (is6Dmask(mask_) && isHandleOnFreeflyer (*this)) {
	return constraints::explicit_::RelativePose::create
	  (n, robot (), gripper->joint (), joint (),
	   gripper->objectPositionInJoint (), localPosition(),
           mask_, ComparisonTypes_t (6, constraints::EqualToZero));
      }
      return constraints::implicit::RelativePose::create
        (n, robot (), gripper->joint (), joint (),
         gripper->objectPositionInJoint (), localPosition(),
         mask_, ComparisonTypes_t (maskSize (mask_), constraints::EqualToZero));
    }

    ImplicitPtr_t Handle::createGraspComplement
    (const GripperPtr_t& gripper, std::string n) const
    {
      if (n.empty()) {
        std::vector<bool> Cmask = complementMask(mask_);
        n = gripper->name() + "_grasps_" + name() + "/complement_" +
          maskToStr (Cmask);
      }
      core::DevicePtr_t r = robot();
      if (is6Dmask(mask_)) {
        return Implicit::create (
            boost::shared_ptr <ZeroDiffFunc> (new ZeroDiffFunc (
                r->configSize(), r->numberDof (), n))
            );
      } else {
        std::vector<bool> Cmask = complementMask(mask_);
        return constraints::implicit::RelativePose::create
          (n, r, gripper->joint (), joint (),
           gripper->objectPositionInJoint (), localPosition(),
           Cmask, ComparisonTypes_t (maskSize (Cmask), constraints::Equality));
      }
    }

    ImplicitPtr_t Handle::createGraspAndComplement
    (const GripperPtr_t& gripper, std::string n) const
    {
      using boost::assign::list_of;
      using core::ExplicitRelativeTransformation;
      if (n.empty()) {
        n = gripper->name() + "_holds_" + name();
      }
      // Create the comparison operator
      assert (mask_.size () == 6);
      ComparisonTypes_t comp (6);
      for (std::size_t i=0; i<6;++i) {
        if (mask_ [i]) {
          comp [i] = constraints::EqualToZero;
        } else {
          comp [i] = constraints::Equality;
        }
      }
      // If handle is on a freeflying object, create an explicit constraint
      if (isHandleOnFreeflyer (*this)) {
	return constraints::explicit_::RelativePose::create
	  (n, robot (), gripper->joint (), joint (),
	   gripper->objectPositionInJoint (), localPosition(),
           std::vector <bool> (6, true), comp);
      }
      return constraints::implicit::RelativePose::create
        (n, robot (), gripper->joint (), joint (),
         gripper->objectPositionInJoint (), localPosition(),
         std::vector <bool> (true, 6), comp);
    }

    ImplicitPtr_t Handle::createPreGrasp
    (const GripperPtr_t& gripper, const value_type& shift, std::string n) const
    {
      Transform3f transform = gripper->objectPositionInJoint ()
        * Transform3f (I3, vector3_t (shift,0,0));
      if (n.empty())
        n = "Pregrasp_ " + maskToStr(mask_) + "_" + name ()
          + "_" + gripper->name ();
      ImplicitPtr_t result
        (constraints::implicit::RelativePose::create
         (n, robot(), gripper->joint (), joint (),
          transform, localPosition(), mask_, ComparisonTypes_t
          (maskSize (mask_), constraints::EqualToZero)));
      return result;
    }

    HandlePtr_t Handle::clone () const
    {
      HandlePtr_t other = Handle::create (name (), localPosition (), robot(), joint ());
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
