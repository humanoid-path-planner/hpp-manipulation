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

#include <hpp/core/distance.hh>
#include <hpp/core/edge.hh>
#include <hpp/manipulation/graph/state.hh>
#include <hpp/manipulation/graph/statistics.hh>
#include <hpp/manipulation/leaf-connected-comp.hh>
#include <hpp/manipulation/roadmap-node.hh>
#include <hpp/manipulation/roadmap.hh>
#include <hpp/util/pointer.hh>

namespace hpp {
namespace manipulation {
Roadmap::Roadmap(const core::DistancePtr_t& distance,
                 const core::DevicePtr_t& robot)
    : core::Roadmap(distance, robot), weak_() {}

RoadmapPtr_t Roadmap::create(const core::DistancePtr_t& distance,
                             const core::DevicePtr_t& robot) {
  Roadmap* ptr = new Roadmap(distance, robot);
  RoadmapPtr_t shPtr(ptr);
  ptr->init(shPtr);
  return shPtr;
}

void Roadmap::clear() {
  Parent::clear();
  Histograms_t::const_iterator it;
  for (it = histograms_.begin(); it != histograms_.end(); ++it) (*it)->clear();
  if (graph_) {
    const Histograms_t& hs = graph_->histograms();
    for (it = hs.begin(); it != hs.end(); ++it) (*it)->clear();
  }
  leafCCs_.clear();
}

void Roadmap::push_node(const core::NodePtr_t& n) {
  Parent::push_node(n);
  const RoadmapNodePtr_t& node = static_cast<RoadmapNodePtr_t>(n);
  statInsert(node);
  leafCCs_.insert(node->leafConnectedComponent());
}

void Roadmap::statInsert(const RoadmapNodePtr_t& n) {
  Histograms_t::const_iterator it;
  for (it = histograms_.begin(); it != histograms_.end(); ++it) (*it)->add(n);
  if (graph_) {
    const Histograms_t& hs = graph_->histograms();
    for (it = hs.begin(); it != hs.end(); ++it) (*it)->add(n);
  }
}

void Roadmap::insertHistogram(const graph::HistogramPtr_t hist) {
  histograms_.push_back(hist);
  core::Nodes_t::const_iterator _node;
  for (_node = nodes().begin(); _node != nodes().end(); ++_node)
    hist->add(static_cast<RoadmapNodePtr_t>(*_node));
}

void Roadmap::constraintGraph(const graph::GraphPtr_t& graph) {
  graph_ = graph;
  // FIXME Add the current nodes() to the graph->histograms()
  // The main issue is that new histograms may be added to
  // graph->histograms() and this class will not know it.
}

RoadmapNodePtr_t Roadmap::nearestNodeInState(
    const ConfigurationPtr_t& configuration,
    const ConnectedComponentPtr_t& connectedComponent,
    const graph::StatePtr_t& state, value_type& minDistance) const {
  core::NodePtr_t result = NULL;
  minDistance = std::numeric_limits<value_type>::infinity();
  const RoadmapNodes_t& roadmapNodes =
      connectedComponent->getRoadmapNodes(state);
  // std::cout << "State: "  << state->name () << std::endl;
  // std::cout << "roadmapNodes.size () = " << roadmapNodes.size ()
  // 		<< std::endl;
  for (RoadmapNodes_t::const_iterator itNode = roadmapNodes.begin();
       itNode != roadmapNodes.end(); ++itNode) {
    value_type d = (*distance())(*(*itNode)->configuration(), *configuration);
    if (d < minDistance) {
      minDistance = d;
      result = *itNode;
    }
  }
  return static_cast<RoadmapNode*>(result);
}

core::NodePtr_t Roadmap::createNode(const ConfigurationPtr_t& q) const {
  // call RoadmapNode constructor with new manipulation connected component
  RoadmapNodePtr_t node = new RoadmapNode(q, ConnectedComponent::create(weak_));
  LeafConnectedCompPtr_t sc = WeighedLeafConnectedComp::create(weak_.lock());
  node->leafConnectedComponent(sc);
  sc->setFirstNode(node);
  return node;
}

graph::StatePtr_t Roadmap::getState(RoadmapNodePtr_t node) {
  return graph_->getState(node);
}

void Roadmap::connect(const LeafConnectedCompPtr_t& cc1,
                      const LeafConnectedCompPtr_t& cc2) {
  if (cc1->canReach(cc2)) return;
  LeafConnectedComp::LeafConnectedComps_t cc2Tocc1;
  if (cc2->canReach(cc1, cc2Tocc1)) {
    merge(cc1, cc2Tocc1);
  } else {
    cc1->to_.insert(cc2.get());
    cc2->from_.insert(cc1.get());
  }
}

void Roadmap::merge(const LeafConnectedCompPtr_t& cc1,
                    LeafConnectedComp::LeafConnectedComps_t& ccs) {
  for (LeafConnectedComp::LeafConnectedComps_t::iterator itcc = ccs.begin();
       itcc != ccs.end(); ++itcc) {
    if (*itcc != cc1.get()) {
      cc1->merge((*itcc)->self());
#ifndef NDEBUG
      std::size_t nb =
#endif
          leafCCs_.erase((*itcc)->self());
      assert(nb == 1);
    }
  }
}

void Roadmap::impl_addEdge(const core::EdgePtr_t& edge) {
  Parent::impl_addEdge(edge);
  const RoadmapNodePtr_t& f = static_cast<RoadmapNodePtr_t>(edge->from());
  const RoadmapNodePtr_t& t = static_cast<RoadmapNodePtr_t>(edge->to());
  if (f->graphState() == t->graphState()) {
    LeafConnectedCompPtr_t cc1(f->leafConnectedComponent());
    LeafConnectedCompPtr_t cc2(t->leafConnectedComponent());

    connect(cc1, cc2);
  }
}
}  // namespace manipulation
}  // namespace hpp
