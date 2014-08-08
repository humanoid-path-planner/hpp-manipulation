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
  GraphComponents components;
  GraphPtr_t graph;
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
}

BOOST_AUTO_TEST_CASE (GraphStructure)
{
  using namespace hpp_test;
  using hpp_test::graph;
  DevicePtr_t dev = Device::create ("test-ur5");
  hpp::model::urdf::loadUrdfModel (dev, "planar", "ur_description", "ur5_robot");
  Devices_t devs; devs.push_back (dev);
  robot = Robot::create ("test-robot", devs, Objects_t());
  graph = Graph::create (robot); components.push_back(graph);
  ns1 = graph->createNodeSelector(); components.push_back(ns1);
  ns2 = graph->createNodeSelector(); components.push_back(ns2);
  n11 = ns1->createNode (); components.push_back(n11);
  n12 = ns1->createNode (); components.push_back(n12);
  n21 = ns2->createNode (); components.push_back(n21);
  n22 = ns2->createNode (); components.push_back(n22);
  e111 = n11->linkTo (n11); components.push_back(e111);
  e121 = n12->linkTo (n11); components.push_back(e121);
  e112 = n11->linkTo (n12); components.push_back(e112);
  e122 = n12->linkTo (n12); components.push_back(e122);
  e211 = n21->linkTo (n21); components.push_back(e211);
  e221 = n22->linkTo (n21); components.push_back(e221);
  e212 = n21->linkTo (n22); components.push_back(e212);
  e222 = n22->linkTo (n22); components.push_back(e222);

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
                        possibleEdges = graph->getEdge (from, to);
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
  Nodes_t nodes = graph->getNode (cfg);
  BOOST_CHECK (nodes.size() == 2);
  BOOST_CHECK (nodes[0] == n11);
  BOOST_CHECK (nodes[1] == n21);
  Edges_t edges = graph->chooseEdge (nodes);
  BOOST_CHECK (edges.size() == 2);
  BOOST_CHECK (edges[0]->from() == n11);
  BOOST_CHECK (edges[1]->from() == n21);
}

BOOST_AUTO_TEST_CASE (ConstraintSets)
{
  using namespace hpp_test;
  using namespace hpp::constraints;
  RelativeComPtr_t com = RelativeCom::create (robot, robot->rootJoint(), vector3_t(0,0,0));
}
