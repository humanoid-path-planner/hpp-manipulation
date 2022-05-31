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

#include "hpp/manipulation/graph/statistics.hh"

#include "hpp/manipulation/constraint-set.hh"

namespace hpp {
namespace manipulation {
namespace graph {
LeafBin::LeafBin(const vector_t& v, value_type* thr)
    : value_(v), nodes_(), thr_(thr) {}

void LeafBin::push_back(const RoadmapNodePtr_t& n) { nodes_.push_back(n); }

bool LeafBin::operator<(const LeafBin& rhs) const {
  const vector_t& v = rhs.value();
  assert(value_.size() == v.size());
  for (int p = 0; p < value_.size(); p++) {
    if (std::abs(value_[p] - v[p]) >= *thr_) return value_[p] < v[p];
  }
  return false;
}

bool LeafBin::operator==(const LeafBin& rhs) const {
  const vector_t& v = rhs.value();
  assert(value_.size() == v.size());
  for (int p = 0; p < value_.size(); p++) {
    if (std::abs(value_[p] - v[p]) >= *thr_) return false;
  }
  return true;
}

const vector_t& LeafBin::value() const { return value_; }

std::ostream& LeafBin::print(std::ostream& os) const {
  Parent::print(os) << " (";
  /// Sort by connected component.
  typedef std::list<RoadmapNodes_t> NodesList_t;
  NodesList_t l;
  bool found;
  for (RoadmapNodes_t::const_iterator itn = nodes_.begin(); itn != nodes_.end();
       ++itn) {
    found = false;
    for (NodesList_t::iterator itc = l.begin(); itc != l.end(); ++itc) {
      if ((*itn)->connectedComponent() == itc->front()->connectedComponent()) {
        itc->push_back(*itn);
        found = true;
        break;
      }
    }
    if (!found) {
      l.push_back(RoadmapNodes_t(1, *itn));
    }
  }
  for (NodesList_t::iterator itc = l.begin(); itc != l.end(); ++itc)
    os << itc->front()->connectedComponent() << " - " << itc->size() << ", ";
  return os << ").";
}

std::ostream& LeafBin::printValue(std::ostream& os) const {
  os << "LeafBin (";
  size_t s = value_.size();
  if (s != 0) {
    for (size_t i = 0; i < s - 1; i++) os << value_[i] << "; ";
    os << value_[s - 1];
  }
  os << ")";
  return os;
}

NodeBin::NodeBin(const StatePtr_t& n) : state_(n), roadmapNodes_() {}

void NodeBin::push_back(const RoadmapNodePtr_t& n) {
  roadmapNodes_.push_back(n);
}

bool NodeBin::operator<(const NodeBin& rhs) const {
  return state()->id() < rhs.state()->id();
}

bool NodeBin::operator==(const NodeBin& rhs) const {
  return state() == rhs.state();
}

const StatePtr_t& NodeBin::state() const { return state_; }

std::ostream& NodeBin::print(std::ostream& os) const {
  Parent::print(os) << " (";
  /// Sort by connected component.
  typedef std::list<RoadmapNodes_t> NodesList_t;
  NodesList_t l;
  bool found;
  for (RoadmapNodes_t::const_iterator itn = roadmapNodes_.begin();
       itn != roadmapNodes_.end(); ++itn) {
    found = false;
    for (NodesList_t::iterator itc = l.begin(); itc != l.end(); ++itc) {
      if ((*itn)->connectedComponent() == itc->front()->connectedComponent()) {
        itc->push_back(*itn);
        found = true;
        break;
      }
    }
    if (!found) {
      l.push_back(RoadmapNodes_t(1, *itn));
    }
  }
  for (NodesList_t::iterator itc = l.begin(); itc != l.end(); ++itc)
    os << itc->front()->connectedComponent() << " - " << itc->size() << ", ";
  return os << ").";
}

std::ostream& NodeBin::printValue(std::ostream& os) const {
  return os << "NodeBin (" << state()->name() << ")";
}

LeafHistogramPtr_t LeafHistogram::create(const Foliation f) {
  return LeafHistogramPtr_t(new LeafHistogram(f));
}

LeafHistogram::LeafHistogram(const Foliation f) : f_(f), threshold_(0) {
  ConfigProjectorPtr_t p = f_.parametrizer()->configProjector();
  if (p) {
    if (p->rightHandSide().size() > 0)
      threshold_ =
          p->errorThreshold() / sqrt((double)p->rightHandSide().size());
  }
}

void LeafHistogram::add(const RoadmapNodePtr_t& n) {
  if (!f_.contains(*n->configuration())) return;
  iterator it = insert(LeafBin(f_.parameter(*n->configuration()), &threshold_));
  it->push_back(n);
  if (numberOfObservations() % 10 == 0) {
    hppDout(info, *this);
  }
}

std::ostream& LeafHistogram::print(std::ostream& os) const {
  os << "Leaf Histogram of foliation " << f_.condition()->name() << std::endl;
  return Parent::print(os);
}

HistogramPtr_t LeafHistogram::clone() const {
  return HistogramPtr_t(new LeafHistogram(f_));
}

StateHistogram::StateHistogram(const graph::GraphPtr_t& graph)
    : graph_(graph) {}

void StateHistogram::add(const RoadmapNodePtr_t& n) {
  iterator it = insert(NodeBin(constraintGraph()->getState(n)));
  it->push_back(n);
  if (numberOfObservations() % 10 == 0) {
    hppDout(info, *this);
  }
}

std::ostream& StateHistogram::print(std::ostream& os) const {
  os << "Graph State Histogram contains: " << std::endl;
  return Parent::print(os);
}

const graph::GraphPtr_t& StateHistogram::constraintGraph() const {
  return graph_;
}

HistogramPtr_t StateHistogram::clone() const {
  return HistogramPtr_t(new StateHistogram(constraintGraph()));
}

unsigned int LeafBin::numberOfObsOutOfConnectedComponent(
    const core::ConnectedComponentPtr_t& cc) const {
  unsigned int count = 0;
  for (RoadmapNodes_t::const_iterator it = nodes_.begin(); it != nodes_.end();
       ++it)
    if ((*it)->connectedComponent() != cc) count++;
  return count;
}

statistics::DiscreteDistribution<RoadmapNodePtr_t>
LeafHistogram::getDistribOutOfConnectedComponent(
    const core::ConnectedComponentPtr_t& cc) const {
  statistics::DiscreteDistribution<RoadmapNodePtr_t> distrib;
  for (const_iterator bin = begin(); bin != end(); ++bin) {
    unsigned int w = bin->numberOfObsOutOfConnectedComponent(cc);
    if (w == 0) continue;
    distrib.insert(bin->nodes().front(), w);
  }
  return distrib;
}

statistics::DiscreteDistribution<RoadmapNodePtr_t> LeafHistogram::getDistrib()
    const {
  statistics::DiscreteDistribution<RoadmapNodePtr_t> distrib;
  for (const_iterator bin = begin(); bin != end(); ++bin) {
    std::size_t w = bin->freq();
    if (w == 0) continue;
    distrib.insert(bin->nodes().front(), w);
  }
  return distrib;
}

const LeafBin::RoadmapNodes_t& LeafBin::nodes() const { return nodes_; }

bool Foliation::contains(ConfigurationIn_t q) const {
  return condition_->isSatisfied(q);
}

vector_t Foliation::parameter(ConfigurationIn_t q) const {
  if (!condition_->isSatisfied(q)) {
    hppDout(error, "Configuration not in the foliation");
  }
  return parametrizer_->configProjector()->rightHandSideFromConfig(q);
}

ConstraintSetPtr_t Foliation::condition() const { return condition_; }

void Foliation::condition(const ConstraintSetPtr_t c) { condition_ = c; }

ConstraintSetPtr_t Foliation::parametrizer() const { return parametrizer_; }

void Foliation::parametrizer(const ConstraintSetPtr_t p) { parametrizer_ = p; }
}  // namespace graph
}  // namespace manipulation
}  // namespace hpp
