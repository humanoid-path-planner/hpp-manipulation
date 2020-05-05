// Copyright (c) 2014, LAAS-CNRS
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

#include <hpp/pinocchio/urdf/util.hh>
#include <hpp/pinocchio/liegroup-element.hh>

#include <hpp/core/steering-method/straight.hh>
#include <hpp/core/path-validation-report.hh>

#include <hpp/constraints/generic-transformation.hh>
#include <hpp/constraints/locked-joint.hh>
#include <hpp/constraints/relative-com.hh>

#include <hpp/manipulation/constraint-set.hh>
#include "hpp/manipulation/graph/state.hh"
#include "hpp/manipulation/graph/state-selector.hh"
#include "hpp/manipulation/graph/graph.hh"
#include "hpp/manipulation/graph/edge.hh"
#include "hpp/manipulation/device.hh"
#include "hpp/manipulation/problem.hh"
#include "hpp/manipulation/graph-path-validation.hh"
#include <hpp/manipulation/steering-method/graph.hh>

#include <boost/test/unit_test.hpp>

using hpp::core::SteeringMethodStraight;
using hpp::core::SteeringMethodPtr_t;

typedef std::vector <hpp::manipulation::graph::GraphComponentPtr_t>
GraphComponents_t;

namespace hpp_test {
  using hpp::core::Configuration_t;
  using hpp::manipulation::graph::GraphPtr_t;
  using hpp::manipulation::graph::StateSelectorPtr_t;
  using hpp::manipulation::graph::StateSelectorPtr_t;
  using hpp::manipulation::graph::StatePtr_t;
  using hpp::manipulation::graph::EdgePtr_t;
  using hpp::manipulation::graph::Graph;
  using hpp::manipulation::graph::GraphComponent;
  using hpp::manipulation::graph::EdgePtr_t;
  using hpp::manipulation::graph::Edges_t;

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

  void initialize (bool ur5)
  {
    components.clear();
    robot = hpp::manipulation::Device::create ("test-robot");
    hpp::manipulation::ProblemPtr_t problem(hpp::manipulation::Problem::create
                                            (robot));
    if (ur5) {
      hpp::pinocchio::urdf::loadModel
        (robot, 0, "ur5/", "anchor",
         "package://example-robot-data/robots/ur_description/urdf/"
         "ur5_joint_limited_robot.urdf",
         "package://example-robot-data/robots/ur_description/srdf/"
         "ur5_joint_limited_robot.srdf");
    }
    SteeringMethodPtr_t sm
      (hpp::manipulation::steeringMethod::Graph::create (*problem));
    hpp::core::ProblemPtr_t pb (problem);
    pb->steeringMethod (sm);

    graph_ = Graph::create ("manipulation-graph", robot, problem);
    components.push_back(graph_);
    graph_->maxIterations (20);
    graph_->errorThreshold (1e-4);
    ns = graph_->createStateSelector("node-selector");
    n1 = ns->createState ("node 1"); components.push_back(n1);
    n2 = ns->createState ("node 2"); components.push_back(n2);
    e11 = n1->linkTo ("edge 11", n1); components.push_back(e11);
    e21 = n2->linkTo ("edge 21", n1); components.push_back(e21);
    e12 = n1->linkTo ("edge 12", n2); components.push_back(e12);
    e22 = n2->linkTo ("edge 22", n2); components.push_back(e22);
    graph_->initialize ();

    q1 = Configuration_t::Zero(6);
    q2 = Configuration_t::Zero(6);
    q1 << 1,1,1,0,2.5,-1.9;
    q2 << 2,0,1,0,2.5,-1.9;
  }
} // namespace hpp_test

BOOST_AUTO_TEST_CASE (GraphStructure)
{
  using namespace hpp_test;
  using hpp_test::graph_;
  initialize (false);

  // Check that GraphComponent keeps track of all object properly.
  size_t index = 0;
  for (GraphComponents_t::iterator it = components.begin();
      it != components.end(); ++it) {
    BOOST_CHECK_MESSAGE (*it == graph_->get (index).lock(),
        "GraphComponent class do not track properly GraphComponents_t inherited objects");
    index++;
  }

  // Test function Graph::getEdge
  StatePtr_t from(n1), to(n2);
  Edges_t checkPossibleEdges,
          possibleEdges = graph_->getEdges (from, to);
  checkPossibleEdges.push_back (e12);
  for (size_t j = 0; j < possibleEdges.size(); j++)
    BOOST_CHECK_MESSAGE (possibleEdges[j] == checkPossibleEdges[j],
        "Possible edge j = " << j);

  Configuration_t cfg;
  StatePtr_t node = graph_->getState (cfg);
  BOOST_CHECK (node == n1);
}

BOOST_AUTO_TEST_CASE (Initialization)
{
  using namespace hpp_test;
  using hpp_test::graph_;
  using hpp::pinocchio::LiegroupElement;
  using hpp::pinocchio::LiegroupSpace;
  using hpp::constraints::LockedJoint;
  using hpp::manipulation::graph::Edge;
  using hpp::manipulation::graph::EdgePtr_t;

  initialize(true);
  hpp::manipulation::DevicePtr_t robot(graph_->robot());
  graph_->addNumericalConstraint(LockedJoint::create
    (robot->jointAt(1), LiegroupElement(LiegroupSpace::R1(true))));
  for (std::size_t i=0; i < components.size(); ++i)
  {
    EdgePtr_t edge(HPP_DYNAMIC_PTR_CAST(Edge, components[i]));
    if (edge) {
      try {
        edge->targetConstraint();
        BOOST_CHECK(false && "should have thrown.");
      } catch (const std::logic_error& exc) {
        std::string msg(exc.what());
        BOOST_CHECK(msg == std::string
                    ("The graph should have been initialized first."));
      }
    }
  }
}
