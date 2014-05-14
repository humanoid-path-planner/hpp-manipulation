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

#include <hpp/model/object-factory.hh>
#include <hpp/manipulation/robot.hh>
#include <hpp/manipulation/grasp.hh>

namespace hpp {
  namespace manipulation {
    void Robot::copyKinematicChain (const JointPtr_t& parentJoint,
				    const JointConstPtr_t& joint)
    {
      JointPtr_t copy = joint->clone ();
      copy->name (joint->robot ()->name () + "/" + joint->name ());
      // The recursivity of the function implies that the kinematic chain is
      // copied in a bottom-up order. For this reason, we need to teach the
      // new joint what is its robot.
      copy->robot (weak_.lock ());
      jointMap_ [joint] = copy;
      parentJoint->addChildJoint (copy, false);
      for (std::size_t i=0; i<joint->numberChildJoints (); ++i) {
	copyKinematicChain (copy, joint->childJoint (i));
      }
    }

    void Robot::copyObject (const JointPtr_t& rootJoint,
			    const ObjectConstPtr_t& object)
    {
      copyKinematicChain (rootJoint, object->rootJoint ());
      copyHandles (object);
    }

    void Robot::copyHandles (const ObjectConstPtr_t& object)
    {
      for (Object::Handles_t::const_iterator itHandle =
	     object->handles ().begin ();
	   itHandle != object->handles ().end (); ++itHandle) {
	Object::Handle handle;
	handle.name = object->name () + "/" + itHandle->name;
	handle.joint = jointMap_ [itHandle->joint];
	handle.localPosition = itHandle->localPosition;
	addHandle (handle.name, handle);
      }
    }

    RobotPtr_t Robot::create (const std::string& name, const Devices_t& robots,
			      const Objects_t& objects)
    {
      Robot* ptr = new Robot (name, robots, objects);
      RobotPtr_t shPtr (ptr);
      ptr->init (shPtr);
      return shPtr;
    }

    ConfigurationIn_t Robot::configuration (const DevicePtr_t& robotOrObject,
					    ConfigurationIn_t fullConfig)
    {
      return fullConfig.segment (rankInConfiguration_ [robotOrObject],
				 robotOrObject->configSize ());
    }

    vectorIn_t Robot::velocity (const DevicePtr_t& robotOrObject,
				vectorIn_t fullVelocity)
    {
      return fullVelocity.segment (rankInVelocity_ [robotOrObject],
				   robotOrObject->numberDof ());
    }

    Robot::Robot (const std::string& name, const Devices_t& robots,
		  const Objects_t& objects) :
      parent_t (name), robots_ (robots), objects_ (objects)
    {
    }

    void Robot::init (const RobotWkPtr_t& self)
    {
      parent_t::init (self);
      weak_ = self;
      buildKinematicChain ();
    }

    /// Iterate over objects and for each new object,
    ///   - copy the kinematic chain,
    ///   - store object in vector.
    void Robot::buildKinematicChain ()
    {
      // Put an anchor joint as root joint
      model::ObjectFactory factory;
      Transform3f pos; pos.setIdentity ();
      rootJoint (factory.createJointAnchor (pos));
      size_type rankInConfiguration = 0;
      size_type rankInVelocity = 0;
      // Copy robot kinematic chains
      for (Devices_t::const_iterator itDev = robots_.begin ();
	   itDev != robots_.end (); ++itDev) {
	copyKinematicChain (rootJoint (), (*itDev)->rootJoint ());
	// Store rank of robot in configuration vector.
	rankInConfiguration_ [(*itDev)] = rankInConfiguration;
	rankInVelocity_ [(*itDev)] = rankInVelocity;
	rankInConfiguration += (*itDev)->configSize ();
	rankInVelocity += (*itDev)->numberDof ();
      }
      // Copy object kinematic chains
      for (Objects_t::const_iterator itObj = objects_.begin ();
	   itObj != objects_.end (); ++itObj) {
	copyObject (rootJoint (), *itObj);
	rankInConfiguration_ [*itObj] = rankInConfiguration;
	rankInVelocity_ [*itObj] = rankInVelocity;
	rankInConfiguration += (*itObj)->configSize ();
	rankInVelocity += (*itObj)->numberDof ();
      }
      // Copy current configuration
      Configuration_t q (configSize ());
      for (Devices_t::const_iterator itDev = robots_.begin ();
	   itDev != robots_.end (); ++itDev) {
	q.segment (rankInConfiguration_ [*itDev],
		   (*itDev)->configSize ()) = (*itDev)->currentConfiguration ();
      }
      for (Objects_t::const_iterator itObj = objects_.begin ();
	   itObj != objects_.end (); ++itObj) {
	q.segment (rankInConfiguration_ [*itObj],
		   (*itObj)->configSize ()) = (*itObj)->currentConfiguration ();
      }
      currentConfiguration (q);
      // Update kinematic chain after copy.
      updateDistances ();
    }

    std::ostream& Robot::print (std::ostream& os) const
    {
      parent_t::print (os);
      // print handles
      os << "Handles:" << std::endl;
      for (Handles_t::const_iterator itHandle = handles_.begin ();
	   itHandle != handles_.end (); ++itHandle) {
	os << "  name: " << itHandle->second.name << std::endl;
	os << "  localPosition: " << itHandle->second.localPosition
	   << std::endl;
	os << "  joint: " << itHandle->second.joint->name () << std::endl;
      }
      return os;
    }

  } // namespace manipulation
} // namespace hpp
