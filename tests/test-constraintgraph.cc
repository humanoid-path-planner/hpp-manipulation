// Copyright (c) 2014, LAAS-CNRS
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

#include <boost/test/unit_test.hpp>
#include <hpp/constraints/generic-transformation.hh>
#include <hpp/constraints/locked-joint.hh>
#include <hpp/constraints/relative-com.hh>
#include <hpp/core/path-validation-report.hh>
#include <hpp/core/steering-method/straight.hh>
#include <hpp/manipulation/constraint-set.hh>
#include <hpp/manipulation/steering-method/graph.hh>
#include <hpp/pinocchio/liegroup-element.hh>
#include <hpp/pinocchio/urdf/util.hh>

#include "hpp/manipulation/device.hh"
#include "hpp/manipulation/graph-path-validation.hh"
#include "hpp/manipulation/graph/edge.hh"
#include "hpp/manipulation/graph/graph.hh"
#include "hpp/manipulation/graph/state-selector.hh"
#include "hpp/manipulation/graph/state.hh"
#include "hpp/manipulation/problem.hh"

using hpp::core::SteeringMethodPtr_t;
using hpp::core::steeringMethod::Straight;

typedef std::vector<hpp::manipulation::graph::GraphComponentPtr_t>
    GraphComponents_t;

namespace hpp_test {
using hpp::core::Configuration_t;
using hpp::manipulation::graph::EdgePtr_t;
using hpp::manipulation::graph::Edges_t;
using hpp::manipulation::graph::Graph;
using hpp::manipulation::graph::GraphComponent;
using hpp::manipulation::graph::GraphPtr_t;
using hpp::manipulation::graph::StatePtr_t;
using hpp::manipulation::graph::StateSelectorPtr_t;

hpp::manipulation::DevicePtr_t robot;
Configuration_t q1, q2;

GraphComponents_t components;
GraphPtr_t graph_;
StateSelectorPtr_t ns;
StatePtr_t n1;
StatePtr_t n2;
EdgePtr_t e11;
EdgePtr_t e21;
EdgePtr_t e12;
EdgePtr_t e22;

void initialize(bool ur5) {
  components.clear();
  robot = hpp::manipulation::Device::create("test-robot");
  hpp::manipulation::ProblemPtr_t problem(
      hpp::manipulation::Problem::create(robot));
  if (ur5) {
    hpp::pinocchio::urdf::loadModel(robot, 0, "ur5/", "anchor",
                                    "package://ur_description/urdf/"
                                    "ur5_joint_limited_robot.urdf",
                                    "package://ur_description/srdf/"
                                    "ur5_joint_limited_robot.srdf");
  }
  SteeringMethodPtr_t sm(
      hpp::manipulation::steeringMethod::Graph::create(problem));
  hpp::core::ProblemPtr_t pb(problem);
  pb->steeringMethod(sm);

  graph_ = Graph::create("manipulation-graph", robot, problem);
  components.push_back(graph_);
  graph_->maxIterations(20);
  graph_->errorThreshold(1e-4);
  ns = graph_->createStateSelector("node-selector");
  n1 = ns->createState("node 1");
  components.push_back(n1);
  n2 = ns->createState("node 2");
  components.push_back(n2);
  e11 = n1->linkTo("edge 11", n1);
  components.push_back(e11);
  e21 = n2->linkTo("edge 21", n1);
  components.push_back(e21);
  e12 = n1->linkTo("edge 12", n2);
  components.push_back(e12);
  e22 = n2->linkTo("edge 22", n2);
  components.push_back(e22);
  graph_->initialize();

  q1 = Configuration_t::Zero(6);
  q2 = Configuration_t::Zero(6);
  q1 << 1, 1, 1, 0, 2.5, -1.9;
  q2 << 2, 0, 1, 0, 2.5, -1.9;
}
}  // namespace hpp_test

BOOST_AUTO_TEST_CASE(GraphStructure) {
  using namespace hpp_test;
  using hpp_test::graph_;
  initialize(false);

  // Check that GraphComponent keeps track of all object properly.
  size_t index = 0;
  for (GraphComponents_t::iterator it = components.begin();
       it != components.end(); ++it) {
    BOOST_CHECK_MESSAGE(*it == graph_->get(index).lock(),
                        "GraphComponent class do not track properly "
                        "GraphComponents_t inherited objects");
    index++;
  }

  // Test function Graph::getEdge
  StatePtr_t from(n1), to(n2);
  Edges_t checkPossibleEdges, possibleEdges = graph_->getEdges(from, to);
  checkPossibleEdges.push_back(e12);
  for (size_t j = 0; j < possibleEdges.size(); j++)
    BOOST_CHECK_MESSAGE(possibleEdges[j] == checkPossibleEdges[j],
                        "Possible edge j = " << j);

  Configuration_t cfg;
  StatePtr_t node = graph_->getState(cfg);
  BOOST_CHECK(node == n1);
}

BOOST_AUTO_TEST_CASE(Initialization) {
  using namespace hpp_test;
  using hpp::constraints::ComparisonTypes_t;
  using hpp::constraints::Equality;
  using hpp::constraints::EqualToZero;
  using hpp::constraints::ImplicitPtr_t;
  using hpp::constraints::LockedJoint;
  using hpp::manipulation::graph::Edge;
  using hpp::manipulation::graph::EdgePtr_t;
  using hpp::pinocchio::LiegroupElement;
  using hpp::pinocchio::LiegroupSpace;
  using hpp_test::graph_;

  initialize(true);
  hpp::manipulation::DevicePtr_t robot(graph_->robot());
  ImplicitPtr_t constraint(LockedJoint::create(
      robot->jointAt(1), LiegroupElement(LiegroupSpace::R1(true))));
  graph_->addNumericalConstraint(constraint);
  // Check that states refuse parameterizable constraints
  constraint->comparisonType(ComparisonTypes_t(1, Equality));
  BOOST_CHECK_THROW(n1->addNumericalConstraint(constraint), std::logic_error);

  for (std::size_t i = 0; i < components.size(); ++i) {
    EdgePtr_t edge(HPP_DYNAMIC_PTR_CAST(Edge, components[i]));
    if (edge) {
      try {
        edge->targetConstraint();
        BOOST_CHECK(false && "should have thrown.");
      } catch (const std::logic_error& exc) {
        std::string msg(exc.what());
        BOOST_CHECK(
            msg ==
            std::string("The graph should have been initialized first."));
      }
    }
  }
}
