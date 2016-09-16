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

#include <hpp/core/steering-method-straight.hh>

#include <hpp/constraints/position.hh>
#include <hpp/constraints/relative-com.hh>

#include "hpp/manipulation/graph/state.hh"
#include "hpp/manipulation/graph/state-selector.hh"
#include "hpp/manipulation/graph/graph.hh"
#include "hpp/manipulation/graph/edge.hh"
#include "hpp/manipulation/device.hh"
#include "hpp/manipulation/problem.hh"
#include "hpp/manipulation/graph-path-validation.hh"

#include <boost/test/unit_test.hpp>

using namespace ::hpp::manipulation;
using hpp::core::SteeringMethodStraight;
using hpp::core::SteeringMethodPtr_t;

typedef std::vector <graph::GraphComponentPtr_t> GraphComponents;

namespace hpp_test {
  DevicePtr_t robot;

  Configuration_t q1, q2;

  GraphComponents components;
  graph::GraphPtr_t graph_;
  graph::NodeSelectorPtr_t ns;
  graph::NodePtr_t n1;
  graph::NodePtr_t n2;
  graph::EdgePtr_t e11;
  graph::EdgePtr_t e21;
  graph::EdgePtr_t e12;
  graph::EdgePtr_t e22;

  void initialize (bool ur5)
  {
    if (ur5) {
#ifdef TEST_UR5
      robot = Device::create ("test-robot");
      hpp::model::urdf::loadUrdfModel (robot, "anchor", "ur_description", "ur5_robot");
#else // TEST_UR5
      BOOST_ERROR ("Set TEST_UR5 in cmake to activate this.");
#endif // TEST_UR5
    } else {
      robot = Device::create ("test-robot");
    }
    SteeringMethodPtr_t sm (SteeringMethodStraight::create (robot));
    graph_ = graph::Graph::create ("manpulation-graph", robot, sm);
    components.push_back(graph_);
    graph_->maxIterations (20);
    graph_->errorThreshold (1e-4);
    ns = graph_->createNodeSelector("node-selector"); components.push_back(ns);
    n1 = ns->createNode ("node 1"); components.push_back(n1);
    n2 = ns->createNode ("node 2"); components.push_back(n2);
    e11 = n1->linkTo ("edge 11", n1); components.push_back(e11);
    e21 = n2->linkTo ("edge 21", n1); components.push_back(e21);
    e12 = n1->linkTo ("edge 12", n2); components.push_back(e12);
    e22 = n2->linkTo ("edge 22", n2); components.push_back(e22);

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
      it != components.end(); ++it) {
    BOOST_CHECK_MESSAGE (*it == graph::GraphComponent::get (index).lock(),
        "GraphComponent class do not track properly GraphComponents inherited objects");
    index++;
  }

  // Test function Graph::getEdge
  graph::NodePtr_t from(n1), to(n2);
  graph::Edges_t checkPossibleEdges,
          possibleEdges = graph_->getEdges (from, to);
  checkPossibleEdges.push_back (e12);
  for (size_t j = 0; j < possibleEdges.size(); j++)
    BOOST_CHECK_MESSAGE (possibleEdges[j] == checkPossibleEdges[j],
        "Possible edge j = " << j);

  Configuration_t cfg;
  graph::NodePtr_t node = graph_->getNode (cfg);
  BOOST_CHECK (node == n1);
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
  n1->addNumericalConstraint (pos, Equality::create ());
}

BOOST_AUTO_TEST_CASE (PathValidationTest)
{
  using namespace hpp_test;

  ProblemPtr_t pb = new Problem (robot);
  BOOST_CHECK(robot->configSize() == 6);
  pb->constraintGraph (graph_);
  ConstraintSetPtr_t constraintn1 = graph_->configConstraint (n1);
  ConstraintSetPtr_t constraintn2 = graph_->configConstraint (n2);
  BOOST_CHECK ( constraintn1->isSatisfied (q1));
  BOOST_CHECK (!constraintn1->isSatisfied (q2));
  BOOST_CHECK ( constraintn2->isSatisfied (q2));
  PathPtr_t p = (*pb->steeringMethod ())(ConfigurationIn_t(q1),ConfigurationIn_t(q2)),
            validp;
  graph::NodePtr_t nq1 = graph_->getNode (q1);
  graph::NodePtr_t nq2 = graph_->getNode (q2);
  BOOST_CHECK (nq1 == n1);
  BOOST_CHECK (nq2 == n2);
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
