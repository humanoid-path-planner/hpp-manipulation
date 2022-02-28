///
/// Copyright (c) 2014 CNRS
/// Authors: Florent Lamiraux
///
///

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.

#include <hpp/manipulation/handle.hh>

#include <pinocchio/multibody/joint/joint-generic.hpp>

#include <hpp/util/debug.hh>

#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/joint-collection.hh>
#include <hpp/pinocchio/gripper.hh>

#include <hpp/constraints/implicit.hh>
#include <hpp/constraints/explicit/relative-pose.hh>
#include <hpp/constraints/generic-transformation.hh>

#include <hpp/manipulation/device.hh>

namespace hpp {
  namespace manipulation {
    using constraints::Implicit;
    using constraints::ImplicitPtr_t;
    using constraints::Explicit;
    using constraints::ExplicitPtr_t;
    using constraints::DifferentiableFunction;

    std::string Handle::className ("Handle");
    namespace {
      static const matrix3_t I3 = matrix3_t::Identity();

      struct ZeroDiffFunc : public DifferentiableFunction {
        ZeroDiffFunc (size_type sIn, size_type sInD,
            std::string name=std::string("Empty function"))
          : DifferentiableFunction (sIn, sInD, LiegroupSpace::empty (), name)
        {
          context ("Grasp complement");
        }

        inline void impl_compute (pinocchio::LiegroupElementRef, vectorIn_t) const {}
        inline void impl_jacobian (matrixOut_t, vectorIn_t) const {}
      };
    }

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

    inline int maskSize (const std::vector<bool>& mask)
    {
      std::size_t res (0);
      for (std::size_t i = 0; i < 6; ++i) {
        if (mask[i]) ++res;
      }
      return (int)res;
    }

    inline std::vector<bool> boolOr(std::vector<bool> mask1,
                                    std::vector<bool> mask2)
    {
      assert(mask1.size() == mask2.size() == 6);
      std::vector<bool> res(mask1.size());
      for (std::size_t i = 0; i < 6; ++i) {
        res[i] = mask1[i] || mask2[i];
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
      maskComp_ = complementMask(mask);
    }

    void Handle::maskComp (const std::vector<bool>& mask)
    {
      assert(maskComp.size() == 6);
      maskComp_ = mask;
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
           6 * constraints::EqualToZero);
      }
      return Implicit::create (RelativeTransformationR3xSO3::create
         (n, robot (), gripper->joint (), joint (),
          gripper->objectPositionInJoint (), localPosition()),
			       6 * constraints::EqualToZero, mask_);
    }

    ImplicitPtr_t Handle::createGraspComplement
    (const GripperPtr_t& gripper, std::string n) const
    {
      if (n.empty()) {
        n = gripper->name() + "_grasps_" + name() + "/complement_" +
          maskToStr (maskComp_);
      }
      core::DevicePtr_t r = robot();
      if (maskSize(maskComp_) == 0) {
        return Implicit::create (
            shared_ptr <ZeroDiffFunc> (new ZeroDiffFunc (
              r->configSize(), r->numberDof (), n)), ComparisonTypes_t());
      } else {
        return  Implicit::create (RelativeTransformationR3xSO3::create
           (n, r, gripper->joint (), joint (),
            gripper->objectPositionInJoint (), localPosition()),
           6 * constraints::Equality, maskComp_);
      }
    }

    ImplicitPtr_t Handle::createGraspAndComplement
    (const GripperPtr_t& gripper, std::string n) const
    {
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
      if (isHandleOnFreeflyer (*this) &&
          maskSize(boolOr(mask_, maskComp_)) == 6) {
	return constraints::explicit_::RelativePose::create
	  (n, robot (), gripper->joint (), joint (),
	   gripper->objectPositionInJoint (), localPosition(), comp);
      }
      return Implicit::create (RelativeTransformationR3xSO3::create
         (n, robot (), gripper->joint (), joint (),
          gripper->objectPositionInJoint (), localPosition()),
                               comp, boolOr(mask_, maskComp_));
    }

    ImplicitPtr_t Handle::createPreGrasp
    (const GripperPtr_t& gripper, const value_type& shift, std::string n) const
    {
      Transform3f M = gripper->objectPositionInJoint ()
        * Transform3f (I3, vector3_t (shift,0,0));
      if (n.empty())
        n = "Pregrasp_ " + maskToStr(mask_) + "_" + name ()
          + "_" + gripper->name ();
      ImplicitPtr_t result (Implicit::create
         (RelativeTransformationR3xSO3::create
          (n, robot(), gripper->joint (), joint (),
           M, localPosition()),
          6 * constraints::EqualToZero, mask_));
      return result;
    }

    HandlePtr_t Handle::clone () const
    {
      HandlePtr_t other = Handle::create (name (), localPosition (), robot(), joint ());
      other->mask(mask_);
      other->mask(maskComp_);
      other->clearance(clearance_);
      return other;
    }

    std::ostream& Handle::print (std::ostream& os) const
    {
      os << "name :" << name () << std::endl;
      os << "local position :" << localPosition () << std::endl;
      os << "joint :" << joint ()->name () << std::endl;
      os << "mask :" << maskToStr (mask()) << std::endl;
      os << "mask complement:" << maskToStr (maskComp_) << std::endl;
      return os;
    }

    std::ostream& operator<< (std::ostream& os, const Handle& handle)
    {
      return handle.print (os);
    }
  } // namespace manipulation
} // namespace hpp
