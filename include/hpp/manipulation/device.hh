///
/// Copyright (c) 2014 CNRS
/// Authors: Florent Lamiraux, Joseph Mirabel
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

#ifndef HPP_MANIPULATION_DEVICE_HH
# define HPP_MANIPULATION_DEVICE_HH

# include <hpp/pinocchio/humanoid-robot.hh>

# include <hpp/core/container.hh>

# include <hpp/manipulation/fwd.hh>
# include <hpp/manipulation/config.hh>

namespace hpp {
  namespace manipulation {
    /// Device with handles.
    ///
    /// As a deriving class of hpp::pinocchio::HumanoidRobot,
    /// it is compatible with hpp::pinocchio::urdf::loadHumanoidRobot
    ///
    /// This class also contains pinocchio::Gripper, Handle and \ref JointAndShapes_t
    class HPP_MANIPULATION_DLLAPI Device : public pinocchio::HumanoidRobot
    {
      public:
        typedef pinocchio::HumanoidRobot Parent_t;

        /// Constructor
        /// \param name of the new instance,
        static DevicePtr_t create (const std::string& name)
        {
          Device* ptr = new Device (name);
          DevicePtr_t shPtr (ptr);
          ptr->init (shPtr);
          return shPtr;
        }

        /// Print object in a stream
        virtual std::ostream& print (std::ostream& os) const;

        void setRobotRootPosition (const std::string& robotName,
                                   const Transform3f& positionWRTParentJoint);

        virtual pinocchio::DevicePtr_t clone () const;

        std::vector<std::string> robotNames () const;

        FrameIndices_t robotFrames (const std::string& robotName) const;

        core::Container <HandlePtr_t> handles;
        core::Container <GripperPtr_t> grippers;
        core::Container <JointAndShapes_t> jointAndShapes;

      protected:
        /// Constructor
        /// \param name of the new instance,
        /// \param robot Robots that manipulate objects,
        /// \param objects Set of objects manipulated by the robot.
        Device (const std::string& name) :
          Parent_t (name)
        {}

        void init (const DeviceWkPtr_t& self)
        {
          Parent_t::init (self);
          self_ = self;
        }

        void initCopy (const DeviceWkPtr_t& self, const Device& other)
        {
          Parent_t::initCopy (self, other);
          self_ = self;
        }

      private:
        DeviceWkPtr_t self_;
    }; // class Device
  } // namespace manipulation
} // namespace hpp
#endif // HPP_MANIPULATION_DEVICE_HH
