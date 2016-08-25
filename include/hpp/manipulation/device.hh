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
    class HPP_MANIPULATION_DLLAPI Device :
      public pinocchio::HumanoidRobot,
      public core::Containers<
        boost::mpl::vector < HandlePtr_t,
                             GripperPtr_t,
                             JointAndShapes_t,
                             JointIndexes_t> >
    {
      public:
        typedef pinocchio::HumanoidRobot Parent_t;

        typedef core::Containers<
          boost::mpl::vector < HandlePtr_t,
          pinocchio::GripperPtr_t,
          JointAndShapes_t,
          JointIndexes_t> > Containers_t;

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

        /// \name Collisions
        /// \{

        /// Add collisions
        /// between the joint vector cache initialized by prepareInsertRobot()
        /// add the current Robot.
        /// When creating a robot from several URDF files, this enables
        /// collisions between joints from different files.
        void didInsertRobot (const std::string& name);

        /// \}

      protected:
        /// Constructor
        /// \param name of the new instance,
        /// \param robot Robots that manipulate objects,
        /// \param objects Set of objects manipulated by the robot.
        Device (const std::string& name) :
          Parent_t (name), jointCacheSize_ (0)
        {}

        void init (const DeviceWkPtr_t& self)
        {
          Parent_t::init (self);
          self_ = self;
        }

      private:
        DeviceWkPtr_t self_;

        std::size_t jointCacheSize_;
    }; // class Device
  } // namespace manipulation
} // namespace hpp
#endif // HPP_MANIPULATION_DEVICE_HH
