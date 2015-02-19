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

#include "hpp/manipulation/handle.hh"

#include <hpp/util/debug.hh>

#include <hpp/model/gripper.hh>
#include <hpp/model/object-factory.hh>

#include "hpp/manipulation/fwd.hh"
#include "hpp/manipulation/object.hh"
#include "hpp/manipulation/robot.hh"

namespace hpp {
  namespace manipulation {
    typedef std::map <std::string, HandlePtr_t> Handles_t;
    typedef std::map <std::string, GripperPtr_t> Grippers_t;
    typedef std::vector<JointPtr_t> JointVector_t;

    const Devices_t& Robot::robots () const
    {
      return robots_;
    }

    const Objects_t& Robot::objects () const
    {
      return objects_;
    }

    const JointPtr_t& Robot::joint (const JointConstPtr_t& original)
    {
      return jointMap_ [original];
    }

    void Robot::addHandle (const std::string& name, const HandlePtr_t& handle)
    {
      handles_ [name] = handle;
    }

    const HandlePtr_t& Robot::handle (const std::string& name) const
    {
      Handles_t::const_iterator it = handles_.find (name);
      if (it == handles_.end ())
        throw std::runtime_error ("no handle with name " + name);
      return it->second;
    }

    void Robot::addGripper (const std::string& name, const GripperPtr_t& gripper)
    {
      grippers_ [name] = gripper;
    }

    const GripperPtr_t& Robot::gripper (const std::string& name) const
    {
      Grippers_t::const_iterator it = grippers_.find (name);
      if (it == grippers_.end ())
        throw std::runtime_error ("no gripper with name " + name);
      return it->second;
    }

    std::vector<std::string> Robot::getDeviceNames()
    {
      std::vector<std::string> deviceNames;
      for ( Devices_t::iterator itDevice = robots_.begin() ; itDevice !=
          robots_.end(); ++itDevice) {
        deviceNames.push_back((*itDevice)->name());
      }
      for ( Objects_t::iterator itObject = objects_.begin() ; itObject !=
          objects_.end(); ++itObject) {
        deviceNames.push_back((*itObject)->name());
      }
      return deviceNames;
    }

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
      addCollisions(object);
    }

    void Robot::copyHandles (const ObjectConstPtr_t& object)
    {
      for (Object::Handles_t::const_iterator itHandle =
	     object->handles ().begin ();
	   itHandle != object->handles ().end (); ++itHandle) {
	HandlePtr_t handle = (*itHandle)->clone ();
	handle->name (object->name () + "/" + (*itHandle)->name ());
	handle->joint (jointMap_ [(*itHandle)->joint ()]);
	addHandle (handle->name (), handle);
      }
    }

    void Robot::copyDevice(const JointPtr_t& rootJoint,
			   const model::DeviceConstPtr_t& device)
    {
      copyKinematicChain (rootJoint, device->rootJoint ());
      copyGrippers (device);
      addCollisions(device);
    }

    void Robot::copyGrippers (const model::DeviceConstPtr_t& device)
    {
      for (model::Grippers_t::const_iterator itGripper =
	     device->grippers ().begin ();
	   itGripper != device->grippers ().end (); ++itGripper) {
	GripperPtr_t gripper = (*itGripper)->clone ();
	gripper->name (device->name () + "/" + (*itGripper)->name ());
	gripper->joint (jointMap_ [(*itGripper)->joint ()]);
        gripper->removeAllDisabledCollisions();
        JointVector_t joints = (*itGripper)->getDisabledCollisions();
        for (model::JointVector_t::const_iterator itJoint = joints.begin() ;
               itJoint != joints.end(); ++itJoint) {
          gripper->addDisabledCollision(jointMap_[*itJoint]);
        }
	addGripper (gripper->name (), gripper);
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

    ConfigurationIn_t Robot::configuration (const model::DevicePtr_t& robotOrObject,
					    ConfigurationIn_t fullConfig)
    {
      return fullConfig.segment (rankInConfiguration_ [robotOrObject],
				 robotOrObject->configSize ());
    }

    vectorIn_t Robot::velocity (const model::DevicePtr_t& robotOrObject,
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
      JointPtr_t root = factory.createJointAnchor (pos);
      root->name (name () + "/root");
      rootJoint (root);
      size_type rankInConfiguration = 0;
      size_type rankInVelocity = 0;
      // Copy robot kinematic chains
      for (Devices_t::const_iterator itDev = robots_.begin ();
	   itDev != robots_.end (); ++itDev) {
	//copyKinematicChain (rootJoint (), (*itDev)->rootJoint ());
        copyDevice(rootJoint (), *itDev);
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

    void Robot::addCollisions (const model::DeviceConstPtr_t& device)
    {
      JointVector_t joints = device->getJointVector ();
      // Cycle through all joint pairs
      for (JointVector_t::const_iterator it1 = joints.begin ();
	   it1 != joints.end (); ++it1) {
	JointPtr_t joint1 = jointMap_[*it1];
        hpp::model::Body* body1 = joint1->linkedBody ();
	  if (!body1) {
	    hppDout (info, "Joint " + joint1->name () <<
		     " has no hpp::model::Body.");
	  } else {
	  for (JointVector_t::const_iterator it2 = getJointVector().begin ();
	        *it2 != jointMap_[*(joints.begin())]; ++it2) {
	    JointPtr_t joint2 = *it2;
            hpp::model::Body* body2 = joint2->linkedBody ();
            if (!body2) {
	    hppDout (info, "Joint " + joint2->name () <<
		     " has no hpp::model::Body.");
	    } else {
	      hppDout (info, "add collision pairs: ("  << joint1->name() <<
                                              "," << joint2->name() << ")");
	      hpp::model::Device::addCollisionPairs (joint1, joint2,
                                                      hpp::model::COLLISION);
	      hpp::model::Device::addCollisionPairs (joint1, joint2,
                                                      hpp::model::DISTANCE);
            }
	  }
        }
      }
    }

    std::ostream& Robot::print (std::ostream& os) const
    {
      parent_t::print (os);
      // print handles
      os << "Handles:" << std::endl;
      for (Handles_t::const_iterator itHandle = handles_.begin ();
	   itHandle != handles_.end (); ++itHandle) {
	os << *itHandle->second;
      }
      return os;
    }
  } // namespace manipulation
} // namespace hpp
