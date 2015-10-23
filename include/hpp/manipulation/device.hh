///
/// Copyright (c) 2014 CNRS
/// Authors: Florent Lamiraux
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

# include <hpp/model/humanoid-robot.hh>

# include "hpp/manipulation/fwd.hh"
# include "hpp/manipulation/config.hh"
# include "hpp/manipulation/container.hh"

namespace hpp {
  namespace manipulation {
    /// Device with handles.
    ///
    /// As a deriving class of hpp::model::HumanoidRobot,
    /// it is compatible with hpp::model::urdf::loadHumanoidRobot
    ///
    /// This class also contains model::Gripper, Handle and \ref JointAndTriangles_t
    class HPP_MANIPULATION_DLLAPI Device : public model::HumanoidRobot,
      public Container <HandlePtr_t>, public Container <model::GripperPtr_t>,
      public Container <JointAndShapes_t>
    {
      public:
        typedef model::HumanoidRobot Parent_t;

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

        /// \name Accessors to container elements
        /// Contained elements are of type model::Gripper, Handle and
        /// \ref JointAndTriangles_t
        /// \{

        /// Get an element of a container
        template <typename Element>
          const Element& get (const std::string& name) const
        {
          return Container <Element>::get (name);
        }

        /// Get the underlying map of a container
        template <typename Element>
          const typename Container<Element>::ElementMap_t& getAll () const
        {
          return Container <Element>::getAll ();
        }

        /// Add an element to a container
        template <typename Element>
          void add (const std::string& name, const Element& element)
        {
          Container <Element>::add (name, element);
        }

        /// \}

        /// \name Collisions
        /// \{

        /// Cache joint vector
        void prepareInsertRobot ()
        {
          didPrepare_ = true;
          jointCache_ = getJointVector ();
        }

        /// Add collisions
        /// between the joint vector cache initialized by prepareInsertRobot()
        /// add the current Robot.
        /// When creating a robot from several URDF files, this enables
        /// collisions between joints from different files.
        void didInsertRobot ();

        /// \}

      protected:
        /// Constructor
        /// \param name of the new instance,
        /// \param robot Robots that manipulate objects,
        /// \param objects Set of objects manipulated by the robot.
        Device (const std::string& name) :
          Parent_t (name), jointCache_ (), didPrepare_ (false)
        {}

        void init (const DeviceWkPtr_t& self)
        {
          Parent_t::init (self);
          self_ = self;
        }

      private:
        DeviceWkPtr_t self_;

        model::JointVector_t jointCache_;
        bool didPrepare_;
    }; // class Device
  } // namespace manipulation
} // namespace hpp
#endif // HPP_MANIPULATION_DEVICE_HH
