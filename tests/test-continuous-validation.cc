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

#define BOOST_TEST_MODULE ContinuousValidation
#include <boost/test/unit_test.hpp>
#include <hpp/core/continuous-validation/progressive.hh>
#include <hpp/manipulation/device.hh>
#include <hpp/manipulation/graph/edge.hh>
#include <hpp/manipulation/graph/graph.hh>
#include <hpp/manipulation/graph/state-selector.hh>
#include <hpp/manipulation/graph/state.hh>
#include <hpp/manipulation/handle.hh>
#include <hpp/manipulation/problem.hh>
#include <hpp/pinocchio/gripper.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/urdf/util.hh>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/spatial/se3.hpp>

BOOST_AUTO_TEST_SUITE(test_hpp_manipulation)

// Test continuous validation when a long object is grasped in a gripper.
BOOST_AUTO_TEST_CASE(continuous_validation_with_constraint) {
  using hpp::manipulation::Device;
  using hpp::manipulation::DevicePtr_t;
  using hpp::manipulation::Handle;
  using hpp::manipulation::HandlePtr_t;
  using hpp::manipulation::matrix3_t;
  using hpp::manipulation::vector3_t;

  using hpp::pinocchio::Gripper;
  using hpp::pinocchio::GripperPtr_t;
  using hpp::pinocchio::Joint;
  using hpp::pinocchio::JointPtr_t;
  using hpp::pinocchio::urdf::loadModelFromString;

  using ::pinocchio::SE3;
  // Load robot with
  //   - one vertical axis,
  //   - a box-shaped link,
  //   - a gripper.
  std::string urdfString =
      "<robot name=\"test\">\n"
      "  <link name=\"base_link\"/>\n"
      "  <link name=\"link_1\">\n"
      "    <collision>\n"
      "      <origin rpy=\"0.5 0 0\" xyz=\"0 0 0\"/>\n"
      "      <geometry>\n"
      "        <box size=\".8 .02 .02\"/>"
      "      </geometry>\n"
      "    </collision>\n"
      "  </link>\n"
      "  <joint name=\"joint_1\" type=\"revolute\">\n"
      "    <limit effort=\"100\" lower=\"-3.2\" upper=\"3.2\" "
      "velocity=\"1\"/>\n"
      "    <axis xyz=\"0 0 1\"/>\n"
      "    <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n"
      "    <parent link=\"base_link\"/>\n"
      "    <child link=\"link_1\"/>\n"
      "  </joint>\n"
      "</robot>\n";
  std::string srdfString =
      "<robot name=\"test\">\n"
      "</robot>\n";
  DevicePtr_t robot(Device::create("test"));
  loadModelFromString(robot, 0, "robot", "anchor", urdfString, srdfString);
  // Load a box-shaped rod with a handle.
  urdfString =
      "<robot name=\"rod\">\n"
      "  <link name=\"base_link\">\n"
      "    <collision>\n"
      "      <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n"
      "      <geometry>\n"
      "        <box size=\"1. .02 .02\"/>\n"
      "      </geometry>\n"
      "    </collision>\n"
      "  </link>\n"
      "</robot>\n";
  srdfString =
      "<robot name=\"rod\">\n"
      "</robot>\n";
  loadModelFromString(robot, 0, "rod", "freeflyer", urdfString, srdfString);
  // Load a spherical obstacle of radius .1 placed at 2.0 along y axis.
  urdfString =
      "<robot name=\"obstacle\">\n"
      "  <link name=\"base_link\">\n"
      "    <collision>\n"
      "      <origin rpy=\"0 0 0\" xyz=\"0 2.0 0\"/>\n"
      "      <geometry>\n"
      "        <sphere radius=\"0.1\"/>\n"
      "      </geometry>\n"
      "    </collision>\n"
      "  </link>\n"
      "</robot>\n";
  srdfString =
      "<robot name=\"obstacle\">\n"
      "</robot>\n";
  loadModelFromString(robot, 0, "obstacle", "anchor", urdfString, srdfString);

  // Create gripper: copy paste code from hpp-manipulation-urdf
  const pinocchio::Model& model = robot->model();
  std::string linkName = "robot/link_1";
  std::string gripperName = "robot/gripper";
  if (!model.existBodyName(linkName))
    throw std::invalid_argument("Link " + linkName +
                                " not found. Cannot create gripper");
  hpp::pinocchio::FrameIndex linkFrameId = model.getFrameId(linkName);
  const ::pinocchio::Frame& grLinkFrame = model.frames[linkFrameId];
  assert(grLinkFrame.type == ::pinocchio::BODY);
  // Gripper position is expressed in link frame. We need to compute
  // the position in joint frame.
  if (model.existFrame(gripperName, ::pinocchio::OP_FRAME))
    throw std::runtime_error("Could not add gripper frame of gripper " +
                             gripperName);
  vector3_t origin;
  origin << -.5, 0, 0;
  matrix3_t I3;
  I3.setIdentity();
  SE3 localPosition(I3, origin);

  robot->model().addFrame(::pinocchio::Frame(
      gripperName, grLinkFrame.parentJoint, linkFrameId,
      grLinkFrame.placement * localPosition, ::pinocchio::OP_FRAME));
  GripperPtr_t gripper =
      Gripper::create(gripperName, robot->shared_from_this());
  gripper->clearance(0.);
  robot->grippers.add(gripper->name(), gripper);

  // Create handle: copy paste code from hpp-manipulation-urdf
  linkName = "rod/base_link";
  if (!model.existBodyName(linkName))
    throw std::invalid_argument("Link \"" + linkName +
                                "\" not found. Cannot create handle");
  const ::pinocchio::Frame& haLinkFrame =
      model.frames[model.getFrameId(linkName)];
  assert(haLinkFrame.type == ::pinocchio::BODY);
  pinocchio::JointIndex index(0);
  std::string jointName("universe");
  JointPtr_t joint(Joint::create(robot, haLinkFrame.parentJoint));
  if (joint) {
    index = joint->index();
    jointName = joint->name();
  }
  // Handle position is expressed in link frame. We need to express it in
  // joint frame.
  origin << 1., 0, 0;
  localPosition = SE3(I3, origin);
  HandlePtr_t handle = Handle::create(
      "robot/handle", haLinkFrame.placement * localPosition, robot, joint);
  handle->clearance(0.);
  handle->mask(std::vector<bool>(6, true));
  vector3_t approachingDirection;
  approachingDirection << 1., 0, 0;
  handle->approachingDirection(approachingDirection);
  robot->handles.add(handle->name(), handle);
  assert(robot->model().existFrame(jointName));
  ::pinocchio::FrameIndex previousFrame(robot->model().getFrameId(jointName));
  robot->model().addFrame(::pinocchio::Frame(
      handle->name(), index, previousFrame,
      haLinkFrame.placement * localPosition, ::pinocchio::OP_FRAME));
  robot->createData();

  using hpp::core::Configuration_t;
  using hpp::core::PathPtr_t;
  using hpp::core::PathValidationReportPtr_t;
  using hpp::core::ValidationReportPtr_t;
  using hpp::core::continuousValidation::Progressive;
  using hpp::manipulation::Problem;
  using hpp::manipulation::ProblemPtr_t;
  using hpp::manipulation::graph::EdgePtr_t;
  using hpp::manipulation::graph::Graph;
  using hpp::manipulation::graph::GraphPtr_t;
  using hpp::manipulation::graph::StatePtr_t;

  ProblemPtr_t problem(Problem::create(robot));
  problem->setPathValidationFactory(Progressive::create, 1e-3);
  GraphPtr_t graph(Graph::create("graph", robot, problem));
  problem->constraintGraph(graph);
  StatePtr_t state(graph->stateSelector()->createState("grasp"));
  state->addNumericalConstraint(handle->createGrasp(gripper, "grasp"));
  EdgePtr_t edge(state->linkTo("rotation", state));
  robot->currentConfiguration(robot->neutralConfiguration());
  graph->initialize();

  JointPtr_t rotationJoint(robot->getJointByName("robot/joint_1"));
  Configuration_t q0(robot->neutralConfiguration()), q1(q0);
  q0[rotationJoint->rankInConfiguration()] = -3.14;
  BOOST_REQUIRE(state->configConstraint()->apply(q0));
  q1 = q0;
  q1[rotationJoint->rankInConfiguration()] = 3.14;
  BOOST_REQUIRE(state->configConstraint()->apply(q1));

  PathPtr_t path;
  BOOST_REQUIRE(edge->build(path, q0, q1));
  ValidationReportPtr_t configReport;
  BOOST_REQUIRE(edge->pathValidation()->validate(q0, configReport));
  BOOST_REQUIRE(edge->pathValidation()->validate(q1, configReport));

  Configuration_t qCollision(robot->configSize());
  BOOST_REQUIRE(path->eval(qCollision, .25 * path->length()));
  BOOST_REQUIRE(!edge->pathValidation()->validate(qCollision, configReport));
  BOOST_REQUIRE(configReport);

  PathPtr_t validPart;
  PathValidationReportPtr_t pathReport;
  BOOST_REQUIRE(
      !edge->pathValidation()->validate(path, false, validPart, pathReport));
  BOOST_REQUIRE(validPart);
  BOOST_REQUIRE(pathReport);
  BOOST_REQUIRE(pathReport->configurationReport);
  BOOST_CHECK_LT(validPart->length(), path->length());
  BOOST_CHECK_GE(pathReport->parameter, path->timeRange().first);
  BOOST_CHECK_LE(pathReport->parameter, path->timeRange().second);
}
BOOST_AUTO_TEST_SUITE_END()
