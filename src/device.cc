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

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/geometry.hpp>

#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/gripper.hh>

#include <hpp/manipulation/handle.hh>

namespace hpp {
  namespace manipulation {
    using se3::Frame;
    using se3::FrameIndex;

    pinocchio::DevicePtr_t Device::clone () const
    {
      Device* ptr = new Device(*this);
      DevicePtr_t shPtr (ptr);
      ptr->initCopy (shPtr, *this);
      return shPtr;
    }

    void Device::setRobotRootPosition (const std::string& rn,
        const Transform3f& t)
    {
      FrameIndexes_t idxs = get<JointIndexes_t> (rn);
      pinocchio::Model& m = model();
      pinocchio::GeomModel& gm = geomModel();
      // The root frame should be the first frame.
      Frame& rootFrame = m.frames[idxs[0]];
      if (rootFrame.type == se3::JOINT) {
        JointIndex jid = m.getJointId (rootFrame.name);
        m.jointPlacements[jid] = t;
        return;
      }

      Transform3f shift (t * rootFrame.placement.inverse());
      // Find all the frames that have the same parent joint.
      for (std::size_t i = 1; i < idxs.size(); ++i) {
        Frame& frame = m.frames[idxs[i]];
        if (frame.parent == rootFrame.parent) {
          // frame is between rootFrame and next moving joints.
          frame.placement = shift * frame.placement;
          if (frame.type == se3::BODY) {
            // Update the geometry object placement.
            for (std::size_t k = 0; k < gm.geometryObjects.size(); ++k)
            {
              se3::GeometryObject& go = gm.geometryObjects[k];
              if (go.parentFrame == idxs[i])
                go.placement = shift * go.placement;
            }
          }
        } else if ((frame.type == se3::JOINT) && (rootFrame.parent == m.parents[frame.parent])) {
          // frame corresponds to a child joint of rootFrame.parent
          m.jointPlacements[frame.parent] = shift * m.jointPlacements[frame.parent];
        }
      }
      invalidate();
    }

    void Device::didInsertRobot (const std::string& name)
    {
      /// Build list of new joints
      std::size_t fvSize = model().frames.size();
      assert (fvSize >= frameCacheSize_);
      FrameIndexes_t newF (fvSize - frameCacheSize_);
      for (std::size_t i = frameCacheSize_; i < fvSize; ++i) {
        assert (
            (model().frames[i].name.compare(0, name.size(), name) == 0)
            && (model().frames[i].name[name.size()] == '/')
            && "Frames have been reordered.");
        newF[i - frameCacheSize_] = i;
      }

      frameCacheSize_ = model().frames.size();
      add (name, newF);
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
