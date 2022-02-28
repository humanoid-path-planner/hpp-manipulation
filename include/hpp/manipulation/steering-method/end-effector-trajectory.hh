// Copyright (c) 2019, LAAS-CNRS
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
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
      typedef shared_ptr < EndEffectorTrajectory > EndEffectorTrajectoryPtr_t;

      using core::PathPtr_t;

      /// Build StraightPath constrained by a varying right hand side constraint.
      class HPP_MANIPULATION_DLLAPI EndEffectorTrajectory
        : public core::SteeringMethod
      {
        public:
          typedef core::interval_t interval_t;

          static EndEffectorTrajectoryPtr_t create
	    (const core::ProblemConstPtr_t& problem)
          {
            EndEffectorTrajectoryPtr_t ptr(new EndEffectorTrajectory (problem));
            ptr->init(ptr);
            return ptr;
          }

          /// Build a trajectory in SE(3).
          /// \param points a Nx7 matrix whose rows corresponds to a pose.
          /// \param weights a 6D vector, weights to be applied when computing
          ///        the distance between two SE3 points.
          static PathPtr_t makePiecewiseLinearTrajectory (matrixIn_t points,
              vectorIn_t weights);

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

          /// Computes an core::InterpolatedPath from the provided interpolation
          /// points.
          /// \param times the time of each configuration
          /// \param configs each column correspond to a configuration
          PathPtr_t projectedPath (vectorIn_t times, matrixIn_t configs) const;

        protected:
          EndEffectorTrajectory (const core::ProblemConstPtr_t& problem)
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
          core::ConstraintSetPtr_t getUpdatedConstraints () const;

          DifferentiableFunctionPtr_t eeTraj_;
          interval_t timeRange_;
          constraints::ImplicitPtr_t constraint_;
      };
    } // namespace steeringMethod
    /// \}
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_STEERING_METHOD_END_EFFECTOR_TRAJECTORY_HH
