///
/// Copyright (c) 2014 CNRS
/// Authors: Florent Lamiraux, Joseph Mirabel
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

#ifndef HPP_MANIPULATION_DEVICE_HH
#define HPP_MANIPULATION_DEVICE_HH

#include <hpp/core/container.hh>
#include <hpp/manipulation/config.hh>
#include <hpp/manipulation/fwd.hh>
#include <hpp/pinocchio/humanoid-robot.hh>

namespace hpp {
namespace manipulation {
/// Device with handles.
///
/// As a deriving class of hpp::pinocchio::HumanoidRobot,
/// it is compatible with hpp::pinocchio::urdf::loadHumanoidRobot
///
/// This class also contains pinocchio::Gripper, Handle and \ref
/// JointAndShapes_t
class HPP_MANIPULATION_DLLAPI Device : public pinocchio::HumanoidRobot {
 public:
  typedef pinocchio::HumanoidRobot Parent_t;

  /// Constructor
  /// \param name of the new instance,
  static DevicePtr_t create(const std::string& name) {
    Device* ptr = new Device(name);
    DevicePtr_t shPtr(ptr);
    ptr->init(shPtr);
    return shPtr;
  }

  DevicePtr_t self() const { return self_.lock(); }

  /// Print object in a stream
  virtual std::ostream& print(std::ostream& os) const;

  void setRobotRootPosition(const std::string& robotName,
                            const Transform3s& positionWRTParentJoint);

  virtual pinocchio::DevicePtr_t clone() const;

  std::vector<std::string> robotNames() const;

  FrameIndices_t robotFrames(const std::string& robotName) const;

  void removeJoints(const std::vector<std::string>& jointNames,
                    Configuration_t referenceConfig);

  core::Container<HandlePtr_t> handles;
  core::Container<GripperPtr_t> grippers;
  core::Container<JointAndShapes_t> jointAndShapes;

 protected:
  /// Constructor
  /// \param name of the new instance,
  /// \param robot Robots that manipulate objects,
  /// \param objects Set of objects manipulated by the robot.
  Device(const std::string& name) : Parent_t(name) {}

  void init(const DeviceWkPtr_t& self) {
    Parent_t::init(self);
    self_ = self;
  }

  void initCopy(const DeviceWkPtr_t& self, const Device& other) {
    Parent_t::initCopy(self, other);
    self_ = self;
  }

  /// For serialization only
  Device() {}

 private:
  DeviceWkPtr_t self_;

  HPP_SERIALIZABLE();
};  // class Device
}  // namespace manipulation
}  // namespace hpp

BOOST_CLASS_EXPORT_KEY(hpp::manipulation::Device)

#endif  // HPP_MANIPULATION_DEVICE_HH
