///
/// Copyright (c) 2015 CNRS
/// Authors: Joseph Mirabel
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

#include <hpp/manipulation/device.hh>

#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/gripper.hh>

#include <hpp/manipulation/handle.hh>

namespace hpp {
  namespace manipulation {
    void Device::didInsertRobot (const std::string& name)
    {
      /// Build list of new joints
      std::size_t jvSize = model().joints.size();
      assert (jvSize >= jointCacheSize_);
      JointIndexes_t newJ (jvSize - jointCacheSize_);
      for (std::size_t i = jointCacheSize_; i < jvSize; ++i) newJ[i] = i;

      jointCacheSize_ = model().joints.size();
      add (name, newJ);
    }

    std::ostream& Device::print (std::ostream& os) const
    {
      Parent_t::print (os);
      // print handles
      os << "Handles:" << std::endl;
      Containers_t::print <HandlePtr_t> (os);
      // print grippers
      os << "Grippers:" << std::endl;
      Containers_t::print <GripperPtr_t> (os);
      return os;
    }

  } // namespace manipulation
} // namespace hpp
