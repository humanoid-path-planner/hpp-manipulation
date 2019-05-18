// Copyright (c) 2019, LAAS-CNRS
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

#ifndef HPP_MANIPULATION_STEERING_METHOD_END_EFFECTOR_TRAJECTORY_HH
#define HPP_MANIPULATION_STEERING_METHOD_END_EFFECTOR_TRAJECTORY_HH

#include <hpp/core/steering-method.hh>

#include <hpp/manipulation/fwd.hh>
#include <hpp/manipulation/config.hh>

namespace hpp {
  namespace manipulation {
    /// \addtogroup steering_method
    /// \{
    namespace steeringMethod {
      HPP_PREDEF_CLASS (EndEffectorTrajectory);
      typedef boost::shared_ptr < EndEffectorTrajectory > EndEffectorTrajectoryPtr_t;

      using core::PathPtr_t;

      /// Build StraightPath constrained by a varying right hand side constraint.
      class HPP_MANIPULATION_DLLAPI EndEffectorTrajectory
        : public core::SteeringMethod
      {
        public:
          typedef core::interval_t interval_t;

          static EndEffectorTrajectoryPtr_t create (const core::Problem& problem)
          {
            EndEffectorTrajectoryPtr_t ptr (new EndEffectorTrajectory (problem));
            ptr->init(ptr);
            return ptr;
          }

          /// Set the constraint whose right hand side will vary.
          void trajectoryConstraint (const constraints::ImplicitPtr_t& ic);

          const constraints::ImplicitPtr_t& trajectoryConstraint ()
          {
            return constraint_;
          }

          /// Set the right hand side of the function from a path
          /// \param se3Output set to True if the output of path must be
          ///                  understood as SE3.
          void trajectory (const PathPtr_t& eeTraj, bool se3Output);

          /// Set the right hand side of the function from another function.
          /// \param eeTraj a function whose input space is of dimension 1.
          /// \param timeRange the input range of eeTraj.
          void trajectory (const DifferentiableFunctionPtr_t& eeTraj, const interval_t& timeRange);

          const DifferentiableFunctionPtr_t& trajectory () const
          {
            return eeTraj_;
          }

          const interval_t& timeRange () const
          {
            return timeRange_;
          }

          core::SteeringMethodPtr_t copy () const
          {
            EndEffectorTrajectoryPtr_t ptr (new EndEffectorTrajectory (*this));
            ptr->init (ptr);
            return ptr;
          }

        protected:
          EndEffectorTrajectory (const core::Problem& problem)
            : core::SteeringMethod (problem)
          {}

          EndEffectorTrajectory (const EndEffectorTrajectory& other)
            : core::SteeringMethod (other),
            eeTraj_ (other.eeTraj_),
            timeRange_ (other.timeRange_),
            constraint_ (other.constraint_)
          {}

          PathPtr_t impl_compute (ConfigurationIn_t q1, ConfigurationIn_t q2) const;

        private:
          DifferentiableFunctionPtr_t eeTraj_;
          interval_t timeRange_;
          constraints::ImplicitPtr_t constraint_;
      };
    } // namespace steeringMethod
    /// \}
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_STEERING_METHOD_END_EFFECTOR_TRAJECTORY_HH
