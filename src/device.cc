///
/// Copyright (c) 2015 CNRS
/// Authors: Joseph Mirabel
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

#include <boost/serialization/weak_ptr.hpp>
#include <hpp/manipulation/device.hh>
#include <hpp/manipulation/handle.hh>
#include <hpp/manipulation/serialization.hh>
#include <hpp/pinocchio/gripper.hh>
#include <hpp/pinocchio/joint-collection.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/util/serialization.hh>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/multibody/model.hpp>

namespace hpp {
namespace manipulation {
using ::pinocchio::Frame;

pinocchio::DevicePtr_t Device::clone() const {
  Device* ptr = new Device(*this);
  DevicePtr_t shPtr(ptr);
  ptr->initCopy(shPtr, *this);
  return shPtr;
}

void Device::setRobotRootPosition(const std::string& rn, const Transform3f& t) {
  FrameIndices_t idxs = robotFrames(rn);
  if (idxs.size() == 0)
    throw std::invalid_argument("No frame for robot name " + rn);

  pinocchio::Model& m = model();
  pinocchio::GeomModel& gm = geomModel();
  // The root frame should be the first frame.
  Frame& rootFrame = m.frames[idxs[0]];
  if (rootFrame.type == ::pinocchio::JOINT) {
    JointIndex jid = m.getJointId(rootFrame.name);
    m.jointPlacements[jid] = t;
    return;
  }

  Transform3f shift(t * rootFrame.placement.inverse());
  // Find all the frames that have the same parent joint.
  for (std::size_t i = 1; i < idxs.size(); ++i) {
    Frame& frame = m.frames[idxs[i]];
    if (frame.parent == rootFrame.parent) {
      // frame is between rootFrame and next moving joints.
      frame.placement = shift * frame.placement;
      if (frame.type == ::pinocchio::BODY) {
        // Update the geometry object placement.
        for (std::size_t k = 0; k < gm.geometryObjects.size(); ++k) {
          ::pinocchio::GeometryObject& go = gm.geometryObjects[k];
          if (go.parentFrame == idxs[i]) go.placement = shift * go.placement;
        }
      }
    } else if ((frame.type == ::pinocchio::JOINT) &&
               (rootFrame.parent == m.parents[frame.parent])) {
      // frame corresponds to a child joint of rootFrame.parent
      m.jointPlacements[frame.parent] = shift * m.jointPlacements[frame.parent];
    }
  }
  invalidate();
}

std::vector<std::string> Device::robotNames() const {
  const pinocchio::Model& model = this->model();
  std::vector<std::string> names;

  for (pinocchio::FrameIndex fi = 1; fi < model.frames.size(); ++fi) {
    const Frame& frame = model.frames[fi];
    std::size_t sep = frame.name.find('/');
    if (sep == std::string::npos) {
      hppDout(warning,
              "Frame " << frame.name << " does not belong to any robots.");
      continue;
    }
    std::string name = frame.name.substr(0, sep);

    if (std::find(names.rbegin(), names.rend(), name) != names.rend())
      names.push_back(name);
  }
  return names;
}

FrameIndices_t Device::robotFrames(const std::string& robotName) const {
  const pinocchio::Model& model = this->model();
  FrameIndices_t frameIndices;

  for (pinocchio::FrameIndex fi = 1; fi < model.frames.size(); ++fi) {
    const std::string& name = model.frames[fi].name;
    if (name.size() > robotName.size() &&
        name.compare(0, robotName.size(), robotName) == 0 &&
        name[robotName.size()] == '/') {
      frameIndices.push_back(fi);
    }
  }
  return frameIndices;
}

void Device::removeJoints(const std::vector<std::string>& jointNames,
                          Configuration_t referenceConfig) {
  Parent_t::removeJoints(jointNames, referenceConfig);

  for (auto& pair : grippers.map)
    pair.second = pinocchio::Gripper::create(pair.second->name(), self_);
  // TODO update handles and jointAndShapes
}

std::ostream& Device::print(std::ostream& os) const {
  Parent_t::print(os);
  // print handles
  os << "Handles:" << std::endl;
  handles.print(os);
  // print grippers
  os << "Grippers:" << std::endl;
  grippers.print(os);
  return os;
}

template <class Archive>
void Device::serialize(Archive& ar, const unsigned int version) {
  using namespace boost::serialization;
  (void)version;
  auto* har = hpp::serialization::cast(&ar);

  ar& make_nvp("base", base_object<pinocchio::HumanoidRobot>(*this));

  // TODO we should throw if a pinocchio::Device instance with name name_
  // and not of type manipulation::Device is found.
  bool written =
      (!har || har->template getChildClass<pinocchio::Device, Device>(
                   name_, false) != this);
  ar& BOOST_SERIALIZATION_NVP(written);
  if (written) {
    ar& BOOST_SERIALIZATION_NVP(self_);
    // TODO (easy) add serialization of core::Container ?
    // ar & BOOST_SERIALIZATION_NVP(handles);
    // ar & BOOST_SERIALIZATION_NVP(grippers);
    // ar & BOOST_SERIALIZATION_NVP(jointAndShapes);
  }
}

HPP_SERIALIZATION_IMPLEMENT(Device);
}  // namespace manipulation
}  // namespace hpp

BOOST_CLASS_EXPORT_IMPLEMENT(hpp::manipulation::Device)
