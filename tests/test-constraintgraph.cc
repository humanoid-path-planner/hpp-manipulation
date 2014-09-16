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

#include <hpp/util/pointer.hh>
#include <hpp/model/urdf/util.hh>

#include <hpp/constraints/position.hh>
#include <hpp/constraints/relative-com.hh>

#include "hpp/manipulation/graph/node.hh"
#include "hpp/manipulation/graph/node-selector.hh"
#include "hpp/manipulation/graph/graph.hh"
#include "hpp/manipulation/graph/edge.hh"
#include "hpp/manipulation/robot.hh"
#include "hpp/manipulation/problem.hh"
#include "hpp/manipulation/graph-path-validation.hh"

#include <boost/test/unit_test.hpp>

using namespace ::hpp::manipulation;
using namespace ::hpp::manipulation::graph;

typedef std::vector <GraphComponentPtr_t> GraphComponents;

namespace hpp_test {
/* 
  enum GraphComponentType {
    UNKNOWN,
    NODE,
    NODE_SELECTOR,
    GRAPH,
    EDGE
  };

  static GraphComponentType getTypeOf (const GraphComponentPtr_t& component) {
    EdgePtr_t e = HPP_DYNAMIC_PTR_CAST (Edge, component);
    NodePtr_t n = HPP_DYNAMIC_PTR_CAST (Node, component);
    GraphPtr_t g = HPP_DYNAMIC_PTR_CAST (Graph, component);
    NodeSelectorPtr_t ns = HPP_DYNAMIC_PTR_CAST (NodeSelector, component);
    if (e)
      return EDGE;
    else if (n)
      return NODE;
    else if (g)
      return GRAPH;
    else if (ns)
      return NODE_SELECTOR;

    return UNKNOWN;
  }
  //*/
  RobotPtr_t robot;

  Configuration_t q1, q2;

  GraphComponents components;
  GraphPtr_t graph_;
  NodeSelectorPtr_t ns1;
  NodeSelectorPtr_t ns2;
  NodePtr_t n11;
  NodePtr_t n12;
  NodePtr_t n21;
  NodePtr_t n22;
  EdgePtr_t e111;
  EdgePtr_t e121;
  EdgePtr_t e112;
  EdgePtr_t e122;
  EdgePtr_t e211;
  EdgePtr_t e221;
  EdgePtr_t e212;
  EdgePtr_t e222;

  void initialize (bool ur5)
  {
    if (ur5) {
#ifdef TEST_UR5
      DevicePtr_t dev = Device::create ("test-ur5");
      hpp::model::urdf::loadUrdfModel (dev, "anchor", "ur_description", "ur5_robot");
      Devices_t devs; devs.push_back (dev);
      robot = Robot::create ("test-robot", devs, Objects_t());
#else // TEST_UR5
      BOOST_ERROR ("Set TEST_UR5 in cmake to activate this.");
#endif // TEST_UR5
    } else {
      robot = Robot::create ("test-robot", Devices_t() , Objects_t());
    }
    graph_ = Graph::create (robot); components.push_back(graph_);
    graph_->maxIterations (20);
    graph_->errorThreshold (1e-4);
    ns1 = graph_->createNodeSelector(); components.push_back(ns1);
    if (!ur5)
      ns2 = graph_->createNodeSelector(); components.push_back(ns2);
    n11 = ns1->createNode (); components.push_back(n11);
    n12 = ns1->createNode (); components.push_back(n12);
    if (!ur5) {
      n21 = ns2->createNode (); components.push_back(n21);
      n22 = ns2->createNode (); components.push_back(n22);
    }
    e111 = n11->linkTo (n11); components.push_back(e111);
    e121 = n12->linkTo (n11); components.push_back(e121);
    e112 = n11->linkTo (n12); components.push_back(e112);
    e122 = n12->linkTo (n12); components.push_back(e122);
    if (!ur5) {
      e211 = n21->linkTo (n21); components.push_back(e211);
      e221 = n22->linkTo (n21); components.push_back(e221);
      e212 = n21->linkTo (n22); components.push_back(e212);
      e222 = n22->linkTo (n22); components.push_back(e222);
    }

    q1 = Configuration_t::Zero(6);
    q2 = Configuration_t::Zero(6);
    q1 << 1,1,1,0,2.5,-1.9;
    q2 << 2,0,1,0,2.5,-1.9;
  }
}

BOOST_AUTO_TEST_CASE (GraphStructure)
{
  using namespace hpp_test;
  using hpp_test::graph_;
  initialize (false);

  // Check that GraphComponent keeps track of all object properly.
  size_t index = 0;
  for (GraphComponents::iterator it = components.begin();
      it != components.end(); it++) {
    BOOST_CHECK_MESSAGE (*it == GraphComponent::get (index).lock(),
        "GraphComponent class do not track properly GraphComponents inherited objects");
    index++;
  }

  // Test function Graph::getEdge
  Nodes_t from, to;
  from.push_back (n11);
  from.push_back (n22);
  to.push_back (n12);
  to.push_back (n22);
  std::vector <Edges_t> checkPossibleEdges,
                        possibleEdges = graph_->getEdge (from, to);
  do {
    Edges_t edges;
    edges.push_back (e112);
    edges.push_back (e222);
    checkPossibleEdges.push_back (edges);
  } while (0);
  for (size_t i = 0; i < possibleEdges.size(); i++) {
    for (size_t j = 0; j < possibleEdges[i].size(); j++)
      BOOST_CHECK_MESSAGE (possibleEdges[i][j] == checkPossibleEdges[i][j],
          "i = " << i << " and j = " << j);
  }

  Configuration_t cfg;
  Nodes_t nodes = graph_->getNode (cfg);
  BOOST_CHECK (nodes.size() == 2);
  BOOST_CHECK (nodes[0] == n11);
  BOOST_CHECK (nodes[1] == n21);
  Edges_t edges = graph_->chooseEdge (nodes);
  BOOST_CHECK (edges.size() == 2);
  BOOST_CHECK (edges[0]->from() == n11);
  BOOST_CHECK (edges[1]->from() == n21);
}

#ifdef TEST_UR5
BOOST_AUTO_TEST_CASE (ConstraintSets)
{
  using namespace hpp_test;
  using namespace hpp::constraints;

  vector_t res, expectedRes;

  initialize (true);
  JointPtr_t ee = robot->getJointByBodyName ("ee_link");

  robot->currentConfiguration (q2);
  robot->computeForwardKinematics ();
  RelativeComPtr_t com = RelativeCom::create (robot, robot->rootJoint(), robot->positionCenterOfMass());
  res.resize (com->outputSize()); expectedRes = vector_t::Zero(res.size());
  (*com) (res,q2);
  BOOST_CHECK (res == expectedRes);

  robot->currentConfiguration (q1);
  robot->computeForwardKinematics ();
  fcl::Vec3f position = ee->currentTransformation ().getTranslation ();
  PositionPtr_t pos = Position::create (robot, ee, vector3_t(0,0,0),
      vector3_t(position[0],position[1],position[2]));
  res.resize (pos->outputSize()); expectedRes = vector_t::Zero(res.size());
  (*pos) (res,q1);
  BOOST_CHECK (res == expectedRes);

  //graph_->addNumericalConstraint (com);
  n11->addNumericalConstraint (pos, Equality::create ());
}

BOOST_AUTO_TEST_CASE (PathValidationTest)
{
  using namespace hpp_test;

  ProblemPtr_t pb = new Problem (robot);
  BOOST_CHECK(robot->configSize() == 6);
  pb->constraintGraph (graph_);
  Nodes_t nodes11; nodes11.push_back (n11);
  Nodes_t nodes12; nodes12.push_back (n12);
  ConstraintSetPtr_t constraintn11 = graph_->configConstraint (nodes11);
  ConstraintSetPtr_t constraintn12 = graph_->configConstraint (nodes12);
  BOOST_CHECK ( constraintn11->isSatisfied (q1));
  BOOST_CHECK (!constraintn11->isSatisfied (q2));
  BOOST_CHECK ( constraintn12->isSatisfied (q2));
  PathPtr_t p = (*pb->steeringMethod ())(ConfigurationIn_t(q1),ConfigurationIn_t(q2)),
            validp;
  Nodes_t nq1 = graph_->getNode (q1);
  Nodes_t nq2 = graph_->getNode (q2);
  BOOST_CHECK (nq1.size() == 1); BOOST_CHECK (nq1[0] == n11);
  BOOST_CHECK (nq2.size() == 1); BOOST_CHECK (nq2[0] == n12);
  GraphPathValidationPtr_t pv = pb->pathValidation ();
  BOOST_CHECK (pv);
  if (!pv->validate(p, false, validp)) {
      BOOST_CHECK_MESSAGE (false,
      "Valid part has length " << validp->length() << " and p " << p->length());
      pv->innerValidation ()->validate (p, false, validp);
      BOOST_CHECK_MESSAGE (false,
      "Inner validation returned: Valid part has length " << validp->length() << " and p " << p->length());
  }
}
#endif // TEST_UR5
