// Copyright (c) 2015, LAAS-CNRS
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//
// This file is part of hpp-manipulation.
// hpp-manipulation is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-manipulation is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-manipulation. If not, see <http://www.gnu.org/licenses/>.

#ifndef HPP_MANIPULATION_CONTAINER_HH
# define HPP_MANIPULATION_CONTAINER_HH

# include <map>
# include <list>
# include <boost/smart_ptr/shared_ptr.hpp>

namespace hpp {
  namespace manipulation {
    template < typename Element, typename Key = std::string>
    class HPP_MANIPULATION_DLLAPI Container
    {
      public:
        typedef std::map <Key, Element> ElementMap_t;

        /// Add an element
        void add (const Key& name, const Element& element)
        {
          map_[name] = element;
        }

        /// Return the element named name
        const Element& get (const Key& name) const
        {
          typename ElementMap_t::const_iterator it = map_.find (name);
          if (it == map_.end ()) return Element ();
          return it->second;
        }

        /// Return a list of all elements
        /// \tparam ReturnType must have a push_back method.
        template <typename ReturnType>
        ReturnType getAllAs () const
        {
          ReturnType l;
          for (typename ElementMap_t::const_iterator it = map_.begin ();
              it != map_.end (); ++it)
            l.push_back (it->second);
          return l;
        }

        /// Return the underlying map.
        const ElementMap_t& getAll () const
        {
          return map_;
        }

        /// Print object in a stream
        std::ostream& print (std::ostream& os) const
        {
          for (typename ElementMap_t::const_iterator it = map_.begin ();
              it != map_.end (); ++it) {
            os << it->first << " : " << it->second << std::endl;
          }
          return os;
        }

      protected:
        /// Constructor
        Container () : map_ ()
        {}

      private:
        ElementMap_t map_;
    }; // class Container
  } // namespace manipulation
} // namespace hpp
#endif // HPP_MANIPULATION_CONTAINER_HH
