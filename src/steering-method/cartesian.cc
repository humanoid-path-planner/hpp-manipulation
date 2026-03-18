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

#include <hpp/constraints/differentiable-function.hh>
#include <hpp/constraints/implicit.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/core/constraint-set.hh>
#include <hpp/core/interpolated-path.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/straight-path.hh>
#include <hpp/manipulation/steering-method/cartesian.hh>
#include <hpp/pinocchio/configuration.hh>
#include <hpp/pinocchio/liegroup-space.hh>
#include <hpp/util/debug.hh>
#include <stdexcept>

namespace hpp {
namespace manipulation {
namespace steeringMethod {

namespace {
typedef core::PathPtr_t PathPtr_t;

template <bool SE3>
class FunctionFromPath : public constraints::DifferentiableFunction {
 public:
  FunctionFromPath(const PathPtr_t& p)
      : DifferentiableFunction(
            1, 1,
            SE3 ? LiegroupSpace::R3xSO3() : LiegroupSpace::Rn(p->outputSize())),
        path_(p) {
    assert(!SE3 || (p->outputSize() == 7 && p->outputDerivativeSize() == 6));
  }

  const PathPtr_t& path() const { return path_; }

  std::ostream& print(std::ostream& os) const {
    return os << (SE3 ? "FunctionFromSE3Path: " : "FunctionFromPath: ")
              << path_->timeRange().first << ", " << path_->timeRange().second;
  }

 protected:
  void impl_compute(core::LiegroupElementRef result, vectorIn_t arg) const {
    bool success = path_->eval(result.vector(), arg[0]);
    if (!success) {
      hppDout(warning, "Failed to evaluate path at param "
                           << arg[0] << incindent << iendl << *path_
                           << decindent);
    }
  }

  void impl_jacobian(matrixOut_t jacobian, vectorIn_t arg) const {
    path_->derivative(jacobian.col(0), arg[0], 1);
  }

 private:
  PathPtr_t path_;
};
}  // namespace

CartesianPtr_t Cartesian::create(const core::ProblemConstPtr_t& problem) {
  CartesianPtr_t ptr(new Cartesian(problem));
  return ptr;
}

PathPtr_t Cartesian::makePiecewiseLinearTrajectory(matrixIn_t points,
                                                   vectorIn_t weights) {
  if (points.cols() != 7)
    throw std::invalid_argument("The input matrix should have 7 columns");
  if (weights.size() != 6)
    throw std::invalid_argument("The weights vector should have 6 elements");
  LiegroupSpacePtr_t se3 = LiegroupSpace::SE3();
  core::PathVectorPtr_t path = core::PathVector::create(7, 6);
  if (points.rows() == 1)
    path->appendPath(core::StraightPath::create(
        se3, points.row(0), points.row(0), interval_t(0, 0)));
  else
    for (size_type i = 1; i < points.rows(); ++i) {
      value_type d = (se3->elementConstRef(points.row(i)) -
                      se3->elementConstRef(points.row(i - 1)))
                         .cwiseProduct(weights)
                         .norm();
      path->appendPath(core::StraightPath::create(
          se3, points.row(i - 1), points.row(i), interval_t(0, d)));
    }
  return path;
}

void Cartesian::maxIterations(size_type iterations) {
  constraints_->configProjector()->maxIterations(iterations);
}
size_type Cartesian::maxIterations() const {
  return constraints_->configProjector()->maxIterations();
}

void Cartesian::errorThreshold(const value_type& threshold) {
  constraints_->configProjector()->errorThreshold(threshold);
}

value_type Cartesian::errorThreshold() const {
  return constraints_->configProjector()->errorThreshold();
}

void Cartesian::trajectoryConstraint(const ImplicitPtr_t& ic) {
  if (trajConstraint_) {
    throw std::logic_error(
        "Cartesian::trajectoryConstraint: trajectory constraint may be called "
        "only once. Recreate a new instance of this class if you want to "
        "change "
        "the constraint.");
  }
  trajConstraint_ = ic->copy();
  constraints_->configProjector()->add(trajConstraint_);
}

void Cartesian::rightHandSide(const PathPtr_t& rhs, bool se3Output) {
  if (se3Output)
    rightHandSide(DifferentiableFunctionPtr_t(new FunctionFromPath<true>(rhs)),
                  rhs->timeRange());
  else
    rightHandSide(DifferentiableFunctionPtr_t(new FunctionFromPath<false>(rhs)),
                  rhs->timeRange());
}

void Cartesian::rightHandSide(const DifferentiableFunctionPtr_t& rhs,
                              const interval_t& timeRange) {
  if (rhs->inputSize() != 1) {
    std::ostringstream os;
    os << "Cartesian::rightHandSide: input space of input function should be 1 "
          "but is "
       << rhs->inputSize() << ".";
    throw std::logic_error(os.str().c_str());
  }
  trajConstraint_->rightHandSideFunction(rhs);
  timeRange_ = timeRange;
  rhs_ = rhs;
}

void Cartesian::checkProblem(const std::string& method) {
  if (!trajConstraint_) {
    std::ostringstream os;
    os << method << ": the trajectory constraint has not been defined.";
    throw std::logic_error(os.str().c_str());
  }
  if (!rhs_) {
    std::ostringstream os;
    os << method
       << ": the right hand side of the trajectory constraint has not been "
          "defined.";
    throw std::logic_error(os.str().c_str());
  }
  if (constraints_->configProjector()->errorThreshold() <= 0) {
    std::ostringstream os;
    os << method
       << ": the error threshold of the numerical solver should be positive, "
          "but is "
       << constraints_->configProjector()->errorThreshold()
       << ". Did you initialize it?";
    throw std::logic_error(os.str().c_str());
  }
  if (constraints_->configProjector()->maxIterations() <= 0) {
    std::ostringstream os;
    os << method
       << ": the maximal number of iteration of the numerical solver should be "
          "positive, but is "
       << constraints_->configProjector()->errorThreshold()
       << ". Did you initialize it?";
    throw std::logic_error(os.str().c_str());
  }
}

bool Cartesian::planPath(ConfigurationIn_t q_init, PathPtr_t& result) {
  checkProblem("Cartesian::planPath");
  bool success = false;
  vector_t times(nDiscreteSteps_ + 1);
  matrix_t steps(trajConstraint_->functionPtr()->inputSize(),
                 nDiscreteSteps_ + 1);

  // Discretize definition interval of the steering method into times
  times[0] = timeRange_.first;
  size_type i = 1;
  for (; i < nDiscreteSteps_; ++i)
    times[i] = timeRange_.first + (double)i *
                                      (timeRange_.second - timeRange_.first) /
                                      (double)nDiscreteSteps_;
  times[nDiscreteSteps_] = timeRange_.second;

  // For each random configuration,
  //   - compute initial configuration of path by projecting the random
  //     configuration (initial configuration for the first time),
  //   - compute following samples by projecting current sample after
  //     updating right hand side.
  // If failure, try next random configuration.
  // Failure can be due to
  //   - projection,
  //   - collision of final configuration,
  //   - validation of path (for collision mainly).
  constraints_->configProjector()->rightHandSideAt(times[0]);
  // Check that initial configuration sarisfies the constraints of the
  // problem
  if (!constraints_->isSatisfied(q_init)) {
    std::ostringstream os;
    os << "Cartesian::planPath: initial configuration "
       << pinocchio::displayConfig(q_init)
       << " does not satisfy the constraints of the problem.";
    throw std::logic_error(os.str().c_str());
  }
  steps.col(0) = q_init;
  success = true;
  for (i = 1; i <= nDiscreteSteps_; ++i) {
    constraints_->configProjector()->rightHandSideAt(times[i]);
    steps.col(i) = steps.col(i - 1);
    if (!constraints_->apply(steps.col(i))) {
      success = false;
      break;
    }
  }
  // return portion of paths that was successfully projected
  if (i > 1) {
    result = projectedPath(times.head(i), steps.leftCols(i));
    return success;
  }
  result = PathPtr_t();
  return false;
}

Cartesian::Cartesian(const core::ProblemConstPtr_t& problem)
    : robot_(problem->robot()),
      constraints_(),
      trajConstraint_(),
      rhs_(),
      timeRange_(),
      nDiscreteSteps_(20) {
  if (!robot_) {
    std::string msg("Cartesian::Cartesian: no robot in problem.");
    throw std::logic_error(msg);
  }
  if (!problem->constraints() || !problem->constraints()->configProjector()) {
    constraints_ = core::ConstraintSet::create(problem->robot(),
                                               "steeringMethod::Cartesian");
    constraints_->addConstraint(ConfigProjector::create(
        problem->robot(), "steeringMethod::Cartesian", 0, 0));
  } else {
    constraints_ = HPP_DYNAMIC_PTR_CAST(core::ConstraintSet,
                                        problem->constraints()->copy());
    assert(constraints_);
  }
}

PathPtr_t Cartesian::projectedPath(vectorIn_t times, matrixIn_t configs) const {
  size_type N = configs.cols();
  if (timeRange_.first != times[0] || timeRange_.second != times[N - 1]) {
    HPP_THROW(std::logic_error, "Cartesian::planPath: Time range ("
                                    << timeRange_.first << ", "
                                    << timeRange_.second
                                    << ") does not match configuration "
                                       "times ("
                                    << times[0] << ", " << times[N - 1]);
  }

  using core::InterpolatedPath;
  using core::InterpolatedPathPtr_t;

  InterpolatedPathPtr_t path = InterpolatedPath::create(
      robot_, configs.col(0), configs.col(N - 1), timeRange_, constraints_);

  for (size_type i = 1; i < configs.cols() - 1; ++i)
    path->insert(times[i], configs.col(i));

  return path;
}

}  // namespace steeringMethod
}  // namespace manipulation
}  // namespace hpp
