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

#include <boost/serialization/weak_ptr.hpp>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/geometry.hpp>

#include <hpp/util/serialization.hh>

#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/joint-collection.hh>
#include <hpp/pinocchio/gripper.hh>

#include <hpp/manipulation/handle.hh>
#include <hpp/manipulation/serialization.hh>

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
      FrameIndices_t idxs = robotFrames (rn);
      if (idxs.size() == 0)
        throw std::invalid_argument ("No frame for robot name " + rn);

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

    std::vector<std::string> Device::robotNames () const
    {
      const pinocchio::Model& model = this->model();
      std::vector<std::string> names;

      for (pinocchio::FrameIndex fi = 1; fi < model.frames.size(); ++fi)
      {
        const Frame& frame = model.frames[fi];
        std::size_t sep = frame.name.find ('/');
        if (sep == std::string::npos) {
          hppDout (warning, "Frame " << frame.name << " does not belong to any robots.");
          continue;
        }
        std::string name = frame.name.substr(0,sep);

        if (std::find(names.rbegin(), names.rend(), name) != names.rend())
          names.push_back (name);
      }
      return names;
    }

    FrameIndices_t Device::robotFrames (const std::string& robotName) const
    {
      const pinocchio::Model& model = this->model();
      FrameIndices_t frameIndices;

      for (pinocchio::FrameIndex fi = 1; fi < model.frames.size(); ++fi)
      {
        const std::string& name = model.frames[fi].name;
        if (   name.size() > robotName.size()
            && name.compare (0, robotName.size(), robotName) == 0
            && name[robotName.size()] == '/') {
          frameIndices.push_back (fi);
        }
      }
      return frameIndices;
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

    template<class Archive>
    void Device::serialize(Archive & ar, const unsigned int version)
    {
      using hpp::serialization::archive_device_wrapper;
      using namespace boost::serialization;

      (void) version;
      ar & make_nvp("base", base_object<pinocchio::HumanoidRobot>(*this));
      archive_device_wrapper* adw = dynamic_cast<archive_device_wrapper*>(&ar);
      bool written = (adw != NULL);
      ar & BOOST_SERIALIZATION_NVP(written);
      if (written) {
        ar & BOOST_SERIALIZATION_NVP(self_);
        // TODO (easy) add serialization of core::Container ?
        //ar & BOOST_SERIALIZATION_NVP(handles);
        //ar & BOOST_SERIALIZATION_NVP(grippers);
        //ar & BOOST_SERIALIZATION_NVP(jointAndShapes);
      }
    }

    HPP_SERIALIZATION_IMPLEMENT(Device);
  } // namespace pinocchio
} // namespace hpp

BOOST_CLASS_EXPORT(hpp::manipulation::Device)
