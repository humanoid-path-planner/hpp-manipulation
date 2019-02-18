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
    using ::pinocchio::Frame;

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
      const FrameIndices_t& idxs = frameIndices.get (rn);
      pinocchio::Model& m = model();
      pinocchio::GeomModel& gm = geomModel();
      // The root frame should be the first frame.
      Frame& rootFrame = m.frames[idxs[0]];
      if (rootFrame.type == ::pinocchio::JOINT) {
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
          if (frame.type == ::pinocchio::BODY) {
            // Update the geometry object placement.
            for (std::size_t k = 0; k < gm.geometryObjects.size(); ++k)
            {
              ::pinocchio::GeometryObject& go = gm.geometryObjects[k];
              if (go.parentFrame == idxs[i])
                go.placement = shift * go.placement;
            }
          }
        } else if ((frame.type == ::pinocchio::JOINT) && (rootFrame.parent == m.parents[frame.parent])) {
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
      FrameIndices_t newF (fvSize - frameCacheSize_);
      for (std::size_t i = frameCacheSize_; i < fvSize; ++i) {
        assert (
            (model().frames[i].name.compare(0, name.size(), name) == 0)
            && (model().frames[i].name[name.size()] == '/')
            && "Frames have been reordered.");
        newF[i - frameCacheSize_] = i;
      }

      frameCacheSize_ = model().frames.size();
      if (frameIndices.has(name)) {
        const FrameIndices_t& old = frameIndices.get(name);
        newF.insert(newF.begin(), old.begin(), old.end());
      }
      frameIndices.add (name, newF);
      createData();
      createGeomData();
    }

    std::ostream& Device::print (std::ostream& os) const
    {
      Parent_t::print (os);
      // print handles
      os << "Handles:" << std::endl;
      handles.print (os);
      // print grippers
      os << "Grippers:" << std::endl;
      grippers.print (os);
      return os;
    }
  } // namespace manipulation
} // namespace hpp
