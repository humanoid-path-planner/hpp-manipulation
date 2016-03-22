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

#include <hpp/model/gripper.hh>
#include <hpp/manipulation/device.hh>
#include <hpp/manipulation/handle.hh>

#include <hpp/model/joint.hh>

namespace hpp {
  namespace manipulation {
        void Device::didInsertRobot (const std::string& name)
        {
          if (!didPrepare_) {
            hppDout (error, "You must call prepareInsertRobot before.");
          }
          didPrepare_ = false;
          /// Build list of new joints
          const model::JointVector_t jv = getJointVector ();
          model::JointVector_t newj;
          newj.reserve (jv.size () - jointCache_.size ());
          model::JointVector_t::const_iterator retFind, it1, it2;
          for (it1 = jv.begin (); it1 != jv.end (); ++it1) {
            retFind = find (jointCache_.begin (), jointCache_.end (), *it1);
            if (retFind == jointCache_.end ())
              newj.push_back (*it1);
          }
          /// Add collision between old joints and new ones.
          for (it1 = newj.begin (); it1 != newj.end (); ++it1) {
            if (!(*it1)->linkedBody ()) continue;
            for (it2 = jointCache_.begin (); it2 != jointCache_.end (); ++it2) {
              if (!(*it2)->linkedBody ()) continue;
              addCollisionPairs (*it1, *it2, model::COLLISION);
              addCollisionPairs (*it1, *it2, model::DISTANCE);
            }
          }
          jointCache_.clear ();
          add (name, newj);
        }
    std::ostream& Device::print (std::ostream& os) const
    {
      Parent_t::print (os);
      // print handles
      os << "Handles:" << std::endl;
      Containers_t::print <HandlePtr_t> (os);
      // print grippers
      os << "Grippers:" << std::endl;
      Containers_t::print <model::GripperPtr_t> (os);
      return os;
    }

  } // namespace manipulation
} // namespace hpp
