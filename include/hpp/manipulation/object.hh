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

# include <hpp/model/device.hh>
# include <hpp/manipulation/handle.hh>
# include <fcl/shape/geometric_shapes.h>

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
      /// List of handles.
      typedef std::vector <HandlePtr_t> Handles_t;

      /// Create instance and return shared pointer.
      static ObjectPtr_t create (const std::string& name)
      {
	Object* ptr = new Object (name);
	ObjectPtr_t shPtr (ptr);
	ptr->init (shPtr);
	return shPtr;
      }

      void addContactTriangle (std::string name, const TriangleList& t)
      {
        contacts_[name] = t;
      }

      TriangleList contactTriangles (std::string name)
      {
        return contacts_ [name];
      }

      const TriangleMap& contactTriangles () const
      {
        return contacts_;
      }

      /// \name Object handles
      /// \{

      /// Add a handle
      void addHandle (const HandlePtr_t& handle)
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
      /// \}
      virtual std::ostream& print (std::ostream &os) const
      {
	parent_t::print (os);
	os << "Handles:" << std::endl;
	for (Handles_t::const_iterator it = handles_.begin ();
	     it != handles_.end (); ++it) {
          os << **it << std::endl;
	}
	return os;
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
      TriangleMap contacts_;
    }; // class Object
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_OBJECT_HH
