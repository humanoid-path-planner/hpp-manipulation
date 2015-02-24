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

#include <hpp/model/joint.hh>

namespace hpp {
  namespace manipulation {
        void Device::didInsertRobot ()
        {
          if (!didPrepare_) {
            hppDout (error, "You must call prepareInsertRobot before.");
          }
          didPrepare_ = false;
          /// Build list of new joints
          const model::JointVector_t jv = getJointVector ();
          model::JointVector_t newj;
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
        }
  } // namespace manipulation
} // namespace hpp
