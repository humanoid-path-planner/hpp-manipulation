//
// Copyright (c) 2014 CNRS
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

#ifndef HPP_MANIPULATION_MOTION_PROJECTOR_HH
# define HPP_MANIPULATION_MOTION_PROJECTOR_HH

# include <hpp/manipulation/config-projector.hh>

namespace hpp {
  namespace manipulation {
    /// Implicit non-linear constraint with offset
    ///
    /// Defined by a list of vector-valued functions and solved numerically
    /// by Newton Raphson like method.
    /// Store locked degrees of freedom for performance optimisation.
    class HPP_MANIPULATION_DLLAPI MotionProjector : public ConfigProjector
    {
    public:
      /// Return shared pointer to new object
      /// \param robot robot the constraint applies to.
      /// \param errorThreshold norm of the value of the constraint under which
      ///        the constraint is considered satified,
      /// \param maxIterations maximal number of iteration in the resolution of
      ///                      the constraint.
      static MotionProjectorPtr_t create (const DevicePtr_t& robot,
                                          const std::string& name,
                                          value_type errorThreshold,
                                          size_type maxIterations);

      /// Add constraint
      void addConstraint (const DifferentiableFunctionPtr_t& constraint);

    protected:
      /// Constructor
      /// \param robot robot the constraint applies to.
      /// \param errorThreshold norm of the value of the constraint under which
      ///        the constraint is considered satified,
      /// \param maxIterations maximal number of iteration in the resolution of
      ///                      the constraint.
      MotionProjector (const DevicePtr_t& robot, const std::string& name,
                       value_type errorThreshold, size_type maxIterations);
      /// Store weak pointer to itself
      void init (const MotionProjectorPtr_t& self)
      {
        ConfigProjector::init (self);
        weak_ = self;
      }
      /// Numerically solve constraint
      virtual bool impl_compute (ConfigurationOut_t configuration);

    private:
      virtual std::ostream& print (std::ostream& os) const;

      void resize ();
      mutable vector_t offset_;
      MotionProjectorWkPtr_t weak_;
    }; // class MotionProjector
  } // namespace manipulation
} // namespace hpp
#endif // HPP_MANIPULATION_MOTION_PROJECTOR_HH
