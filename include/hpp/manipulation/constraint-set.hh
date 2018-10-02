// Copyright (c) 2015 CNRS
// Authors: Joseph Mirabel
//
// This file is part of hpp-manipulation
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
// hpp-manipulation  If not, see
// <http://www.gnu.org/licenses/>.

#ifndef HPP_MANIPULATION_CONSTRAINT_SET_HH
# define HPP_MANIPULATION_CONSTRAINT_SET_HH

# include <hpp/core/constraint-set.hh>

# include <hpp/manipulation/fwd.hh>
# include <hpp/manipulation/graph/fwd.hh>
# include <hpp/manipulation/config.hh>

namespace hpp {
  namespace manipulation {
    /// \addtogroup constraints
    /// \{

    /// a core::ConstraintSet remembering which edge created it
    class HPP_MANIPULATION_DLLAPI ConstraintSet : public core::ConstraintSet
    {
    public:
      typedef core::ConstraintSet Parent_t;

      /// Return shared pointer to new object
      static ConstraintSetPtr_t create (const DevicePtr_t& robot,
					const std::string& name);

      /// Return shared pointer to new object
      static ConstraintSetPtr_t createCopy (const ConstraintSetPtr_t& cs);

      /// return shared pointer to copy
      virtual ConstraintPtr_t copy () const;

      void edge (graph::EdgePtr_t edge);

      graph::EdgePtr_t edge () const;

    protected:
      /// Constructor
      ConstraintSet (const DevicePtr_t& robot, const std::string& name);
      /// Copy constructor
      ConstraintSet (const ConstraintSet& other);
      /// Store weak pointer to itself.
      void init (const ConstraintSetPtr_t& self);

      virtual std::ostream& print (std::ostream& os) const;

    private:
      graph::EdgePtr_t edge_;
      ConstraintSetWkPtr_t weak_;
    }; // class ConstraintSet

    struct ConstraintAndComplement_t {
      ImplicitPtr_t constraint;
      ImplicitPtr_t complement;
      ImplicitPtr_t both;
      ConstraintAndComplement_t (const ImplicitPtr_t& constr,
          const ImplicitPtr_t& comp,
          const ImplicitPtr_t& b) :
        constraint (constr), complement (comp), both (b)
      {
      }
    };
    typedef std::vector <ConstraintAndComplement_t>
      ConstraintsAndComplements_t;
    /// \}
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_CONSTRAINT_SET_HH
