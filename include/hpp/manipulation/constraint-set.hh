// Copyright (c) 2015 CNRS
// Authors: Joseph Mirabel
//

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.

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

      ConstraintSet() {}
      HPP_SERIALIZABLE();
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

BOOST_CLASS_EXPORT_KEY(hpp::manipulation::ConstraintSet)

#endif // HPP_MANIPULATION_CONSTRAINT_SET_HH
