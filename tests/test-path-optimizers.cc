// Copyright (c) 2026, LAAS-CNRS
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

#include <boost/test/unit_test.hpp>
#include <hpp/core/plugin.hh>
#include <hpp/manipulation/device.hh>
#include <hpp/manipulation/graph-optimizer.hh>
#include <hpp/manipulation/path-optimization/enforce-transition-semantic.hh>
#include <hpp/manipulation/path-optimization/random-shortcut.hh>
#include <hpp/manipulation/path-optimization/spline-gradient-based.hh>
#include <hpp/manipulation/problem-solver.hh>
#include <hpp/manipulation/problem.hh>

using hpp::core::path::BernsteinBasis;
using hpp::manipulation::Device;
using hpp::manipulation::Problem;
using hpp::manipulation::ProblemSolver;
namespace pathOptimization = hpp::manipulation::pathOptimization;

template <typename T>
bool createsType(const ProblemSolver& solver, const std::string& name,
                 const hpp::core::ProblemConstPtr_t& problem) {
  return std::dynamic_pointer_cast<T>(
             solver.pathOptimizers.get(name)(problem)) != nullptr;
}

BOOST_AUTO_TEST_CASE(problem_solver_optimizers) {
  ProblemSolver* solver = ProblemSolver::create();
  auto problem = Problem::create(Device::create("robot"));

  BOOST_CHECK(createsType<pathOptimization::RandomShortcut>(
      *solver, "RandomShortcut", problem));
  BOOST_CHECK(createsType<hpp::manipulation::GraphOptimizer>(
      *solver, "Graph-RandomShortcut", problem));
  BOOST_CHECK(createsType<hpp::manipulation::GraphOptimizer>(
      *solver, "Graph-PartialShortcut", problem));
  BOOST_CHECK(createsType<pathOptimization::EnforceTransitionSemantic>(
      *solver, "EnforceTransitionSemantic", problem));
  delete solver;
}

BOOST_AUTO_TEST_CASE(spline_gradient_based_plugin) {
  ProblemSolver* solver = ProblemSolver::create();
  auto problem = Problem::create(Device::create("robot"));
  BOOST_REQUIRE(hpp::core::plugin::loadPlugin(SPLINE_PLUGIN, solver));

  BOOST_CHECK(
      (createsType<pathOptimization::SplineGradientBased<BernsteinBasis, 1>>(
          *solver, "SplineGradientBased_bezier1", problem)));
  BOOST_CHECK(
      (createsType<pathOptimization::SplineGradientBased<BernsteinBasis, 3>>(
          *solver, "SplineGradientBased_bezier3", problem)));
  delete solver;
}
