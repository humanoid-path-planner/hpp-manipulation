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

#ifndef HPP_MANIPULATION_ROBOT_HH
# define HPP_MANIPULATION_ROBOT_HH

# include <hpp/manipulation/handle.hh>
# include <hpp/manipulation/object.hh>
# include <hpp/model/device.hh>

namespace hpp {
  namespace manipulation {
    /// Robot holding one or several objects.
    ///
    /// As a deriving from class hpp::core::Device, objects of this class
    /// store a kinematic chain that contains a copy of the robot and objects
    /// given as arguments to the constructor.
    class HPP_MANIPULATION_DLLAPI Robot : public Device
    {
    public:
      typedef Device parent_t;
      /// Constructor
      /// \param name of the new instance,
      /// \param robot Robots that manipulate objects,
      /// \param objects Set of objects manipulated by the robot.
      static RobotPtr_t create (const std::string& name,
				const Devices_t& robots,
				const Objects_t& objects);

      /// Get configuration of robot or object
      /// \param robotOrObject robot or object,
      /// \param fullConfig configuration of the full kinematic chain,
      /// \return configuration of robot or object
      ConfigurationIn_t configuration (const DevicePtr_t& robotOrObject,
				       ConfigurationIn_t fullConfig);

      /// Get velocity of robot or object
      /// \param robotOrObject robot or object,
      /// \param fullVelocity velocity of the full kinematic chain,
      /// \return velocity of robot or object
      vectorIn_t velocity (const DevicePtr_t& robotOrObject,
			   vectorIn_t fullVelocity);

      /// Get robot manipulating objects
      const Devices_t& robots () const
      {
	return robots_;
      }
      /// Get objects manipulated by the robot
      const Objects_t& objects () const
      {
	return objects_;
      }
      /// Get joint of this corresponding to joint of original object or robot.
      const JointPtr_t& joint (const JointConstPtr_t& original)
      {
	return jointMap_ [original];
      }

      /// \name Composite robot handles
      /// \{

      /// Add a handle
      void addHandle (const std::string& name, const HandlePtr_t& handle)
      {
	handles_ [name] = handle;
      }
      /// Return the handle named name
      const HandlePtr_t& handle (const std::string& name) const
      {
	Handles_t::const_iterator it = handles_.find (name);
	if (it == handles_.end ())
	  throw std::runtime_error ("no handle with name " + name);
	return it->second;
      }
      /// \}

      /// Add a gripper
      void addGripper (const std::string& name, const GripperPtr_t& gripper)
      {
        grippers_ [name] = gripper;
      }
      /// Return the gripper named name
      const GripperPtr_t& gripper (const std::string& name) const
      {
	Grippers_t::const_iterator it = grippers_.find (name);
	if (it == grippers_.end ())
	  throw std::runtime_error ("no gripper with name " + name);
	return it->second;
      }

      /// Get Device names in the same order than the compositeRobot has been
      /// build
      std::vector<std::string> getDeviceNames()
      {
        std::vector<std::string> deviceNames;
        for ( Devices_t::iterator itDevice = robots_.begin() ; itDevice != 
                robots_.end() ; itDevice++ ) {
          deviceNames.push_back((*itDevice)->name());
        }
        for ( Objects_t::iterator itObject = objects_.begin() ; itObject !=
                objects_.end() ; itObject++ ) {
          deviceNames.push_back((*itObject)->name());
        }
        return deviceNames;
      }

      /// Print object in a stream
      virtual std::ostream& print (std::ostream& os) const;
      
    protected:
      /// Constructor
      /// \param name of the new instance,
      /// \param robot Robots that manipulate objects,
      /// \param objects Set of objects manipulated by the robot.
      Robot (const std::string& name, const Devices_t& robots,
	     const Objects_t& objects);
      /// Initialize robot
      ///
      /// Call parent implementation, store weak pointer to itself and copy
      /// the kinematic chain of robots and objects.
      void init (const RobotWkPtr_t& self);
    private:
      typedef std::map <DevicePtr_t, size_type> RankMap_t;
      typedef std::map <std::string, HandlePtr_t> Handles_t;
      typedef std::map <std::string, GripperPtr_t> Grippers_t;
      /// Build the kinematic chain composed of the robot and of the manipulated
      /// objects
      void buildKinematicChain ();
      /// Recursively copy a kinematic chain
      void copyKinematicChain (const JointPtr_t& parentJoint,
			       const JointConstPtr_t& joint);
      /// Copy object including kinematic chain.
      void copyObject (const JointPtr_t& rootJoint,
		       const ObjectConstPtr_t& object);
      /// Copy handles of the object into composite robot.
      void copyHandles (const ObjectConstPtr_t& object);
      /// Copy Device including kinematic chain.
      void copyDevice (const JointPtr_t& rootJoint,
                       const DeviceConstPtr_t& device);
      /// Copy grippers of the device into composite robot.
      void copyGrippers (const DeviceConstPtr_t& device);

      /// Add Collision pairs between the robots
      void addCollisions(const DeviceConstPtr_t& device);

      /// Check if joint1 joint2 is a couple gripper-handle 
      bool isCollisionPairDisabled(JointPtr_t& joint1, JointPtr_t& joint2);

      Devices_t robots_;
      /// Set of objects
      Objects_t objects_;
      /// Map of handles
      Handles_t handles_;
      /// Map of grippers
      Grippers_t grippers_;
      /// Map from original joints to copies
      JointMap_t jointMap_;
      /// Weak pointer to itself
      RankMap_t rankInConfiguration_;
      RankMap_t rankInVelocity_;
      RobotWkPtr_t weak_;
    }; // class Robot
  } // namespace manipulation
} // namespace hpp
#endif // HPP_MANIPULATION_ROBOT_HH
