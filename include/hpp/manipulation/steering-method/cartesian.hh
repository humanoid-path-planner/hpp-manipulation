// Copyright (c) 2026, LAAS-CNRS
// Authors: Florent Lamiraux
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

#ifndef HPP_MANIPULATION_STEERING_METHOD_CARTESIAN_HH
#define HPP_MANIPULATION_STEERING_METHOD_CARTESIAN_HH

#include <hpp/manipulation/fwd.hh>

namespace hpp {
namespace manipulation {
namespace steeringMethod {

HPP_PREDEF_CLASS(Cartesian);
typedef shared_ptr<Cartesian> CartesianPtr_t;

/// Build a robot trajectory from an end-effector trajectory
///
/// This class does not derive from \link hpp::core::SteeringMethod SteeringMethod \endlink since it
/// does not link two configurations by a path. Instead, it only takes an initial configuration
/// and a trajectory of an end-effector.
///
/// To use this class, the user needs to provide
///  \li a constraint with value in \f$SE(3)\f$. An easy way to create such a
///  constraint is to use method hpp::manipulation::Handle::createGrasp. The
///  constraint is passed to this class using method \link
/// Cartesian::trajectoryConstraint trajectoryConstraint \endlink.
///  \li the time-varying right hand side of this constraint along the path
///  the user wants to create in the form of a hpp::core::Path instance
///  with values in \f$SE(3)\f$. For that, \link
/// Cartesian::makePiecewiseLinearTrajectory
/// makePiecewiseLinearTrajectory \endlink method may be useful.
///
/// Once the steering method has been initialized, it can be called with and initial
/// configuration \c q_init. The interval of definition \f$[0,T]\f$
/// of the output path is the same as the one of the path provided as the right
/// hand side of the constraint.
/// Note that \c q_init should satisfy the constraint at times 0.
class Cartesian {

 public:
  typedef constraints::ImplicitPtr_t ImplicitPtr_t;
  typedef core::ConfigurationIn_t ConfigurationIn_t;
  typedef core::interval_t interval_t;
  typedef core::PathPtr_t PathPtr_t;
  
  static CartesianPtr_t create(const core::ProblemConstPtr_t& problem);

  /** Build a path in SE(3).
      \param points a Nx7 matrix whose rows corresponds to poses.
      \param weights a 6D vector, weights to be applied when computing
             the distance between two SE3 points.

      The trajectory \f$T\f$ is defined as follows. Let \f$N\f$ be the number of
      lines of matrix \c points, \f$p_i\f$ be the i-th line of \c points and
      let \f$W\f$ be the
      diagonal matrix with the coefficients of \c weights:
      \f[
      W = \left(\begin{array}{cccccc}
      w_1 & 0 & 0 & 0 & 0 & 0\\
      0 & w_2 & 0 & 0 & 0 & 0\\
      0 & 0 & w_3 & 0 & 0 & 0\\
      0 & 0 & 0 & w_4 & 0 & 0\\
      0 & 0 & 0 & 0 & w_5 & 0\\
      0 & 0 & 0 & 0 & 0 & w_6\\
      \end{array}\right)
      \f]

      \f{eqnarray*}{
        f(t) = \mathbf{p}_i \oplus \frac{t-t_i}{t_{i+1}-t_i}
     (\mathbf{p}_{i+1}-\mathbf{p}_i) && \mbox{ for } t \in [t_i,t_{i+1}] \f}

      where \f$t_0 = 0\f$ and
      \f{eqnarray*}{
       t_{i+1}-t_i = \|W(\mathbf{p}_{i+1}-\mathbf{p}_i)\| && \mbox{for } i
       \mbox{ such that } 1 \leq i \leq N-1
       \f} */
  static PathPtr_t makePiecewiseLinearTrajectory(matrixIn_t points,
                                                 vectorIn_t weights);

  /// Set maximal number of iterations of numerical solver
  void maxIterations(size_type iterations);
  /// Get maximal number of iterations of numerical solver
  size_type maxIterations() const;

  /// Set error threshold of numerical solver
  void errorThreshold(const value_type& threshold);
  /// Get error threshold of numerical solver
  value_type errorThreshold() const;
  /// Set the trajectory constraint
  void trajectoryConstraint(const ImplicitPtr_t& ic);
  /// Get the trajectory constraint
  const ImplicitPtr_t& trajectoryConstraint() {
    return trajConstraint_;
  }
  /// Set the right hand side of the trajectory constraint from a path
  /// \param rhs function from an interval to SE(3).
  /// \param se3Output set to True if the output of path must be
  ///                  understood as SE3.
  void rightHandSide(const PathPtr_t& rhs, bool se3Output);

  /// Set the right hand side of the function from another function.
  /// \param rhs a function whose input space is of dimension 1.
  /// \param timeRange the input range of eeTraj.
  void rightHandSide(const DifferentiableFunctionPtr_t& rhs,
                     const interval_t& timeRange);

  /// Get time-varying right hand side of trajectory constraint
  const DifferentiableFunctionPtr_t& rightHandSide() const { return rhs_; }

  /// Get interval of definition of right hand side of trajectory constraint
  const interval_t& timeRange() const { return timeRange_; }

  /// Number of steps to generate goal config (successive projections).
  size_type nDiscreteSteps() const { return nDiscreteSteps_; }

  void nDiscreteSteps(size_type n) {
    assert(n > 0);
    nDiscreteSteps_ = n;
  }


  /// \brief Plan a path starting from an initial configuration
  ///
  /// \param q_init initial configuration
  ///
  /// \retval result the resulting path in case of success, a valid portion of path satisfying
  ///         the trajectory constraint along a sub-interval starting at 0 otherwise.
  /// \return true if the path is successfully computed, false otherwise
  ///
  /// The interval of definition is discretized into a number of sub-intervals defined by
  /// method \link Cartesian::nDiscreteSteps
  /// nDiscreteSteps\endlink. For each discretized value, a configuration is computed by
  /// projecting
  /// the previous one (or the initial configuration for the first discretized value)
  /// onto the time-varying constraint.
  ///
  /// In case of failure, the interpolated path until the last successful
  /// projection is returned.
  ///
  /// \note No path validation is performed. Collision checking should be performed on the output
  ///       of this method.
  bool planPath(ConfigurationIn_t q_init, PathPtr_t& result);

 protected:
  /// Store the constraints of the problem
  Cartesian(const core::ProblemConstPtr_t& problem);
  PathPtr_t projectedPath(vectorIn_t times, matrixIn_t configs) const;
  void checkProblem(const std::string& method);
 private:
  /// Robot
  pinocchio::DevicePtr_t robot_;
  /// Constraints the system is subjected to, provided by the problem
  core::ConstraintSetPtr_t constraints_;
  /// Constraint that defined the motion of the end-effector
  ImplicitPtr_t trajConstraint_;
  DifferentiableFunctionPtr_t rhs_;
  interval_t timeRange_;
  /// Number of steps along the definition interval to project the configurations
  size_type nDiscreteSteps_;

}; // class Cartesian

/// \addtogroup steering_method
/// \{


/// \}

}  // namespace steeringMethod
}  // namespace manipulation
}  // namespace hpp

#endif // HPP_MANIPULATION_STEERING_METHOD_CARTESIAN_HH
