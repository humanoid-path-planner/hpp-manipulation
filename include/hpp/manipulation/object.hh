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

#ifndef HPP_MANIPULATION_OBJECT_HH
# define HPP_MANIPULATION_OBJECT_HH

# include <hpp/manipulation/config.hh>
# include <hpp/manipulation/fwd.hh>
# include <hpp/model/device.hh>

namespace hpp {
  namespace manipulation {
    /// Object to be manipulated
    /// 
    /// Base class for any object aimed at being manipulated by a robot. It can
    /// be a rigid object like a screw driver or an articulated object like a
    /// door. For this reason the class derives from model::Device.
    ///
    /// An object can have some handles defined by the local position in a
    /// given joint frame.
    class HPP_MANIPULATION_DLLAPI Object : public model::Device
    {
    public:
      typedef model::Device parent_t;
      /// Part of an object that is aimed at being grasped
      struct Handle
      {
	std::string name;
	/// Position of the handle in the joint frame.
	Transform3f localPosition;
	/// Joint to which the handle is linked.
	JointPtr_t joint;
      }; // struct Handle
      /// List of handles.
      typedef std::vector <Handle> Handles_t;

      /// Create instance and return shared pointer.
      static ObjectPtr_t create (const std::string& name)
      {
	Object* ptr = new Object (name);
	ObjectPtr_t shPtr (ptr);
	ptr->init (shPtr);
	return shPtr;
      }
      /// Add a handle
      void addHandle (const Handle& handle)
      {
	handles_.push_back (handle);
      }
      /// Return list of handles of the object
      Handles_t& handles ()
      {
	return handles_;
      }
      /// Return list of handles of the object
      const Handles_t& handles () const
      {
	return handles_;
      }
    protected:
      Object (const std::string& name) : parent_t (name), handles_ ()
	{
	}
      void init (const ObjectWkPtr_t& weakPtr)
      {
	parent_t::init (weakPtr);
	weakPtr_ = weakPtr_;
      }
    private:
      Handles_t handles_;
      ObjectWkPtr_t weakPtr_;
    }; // class Object
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_OBJECT_HH
