// Copyright (c) 2015, Joseph Mirabel
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

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/weak_ptr.hpp>
#include <hpp/manipulation/device.hh>
#include <hpp/manipulation/graph/edge.hh>
#include <hpp/manipulation/graph/graph.hh>
#include <hpp/manipulation/roadmap-node.hh>
#include <hpp/manipulation/serialization.hh>
#include <hpp/manipulation/weighed-distance.hh>
#include <hpp/pinocchio/serialization.hh>
#include <hpp/util/debug.hh>
#include <hpp/util/serialization.hh>

namespace hpp {
namespace manipulation {
WeighedDistancePtr_t WeighedDistance::create(const DevicePtr_t& robot,
                                             const graph::GraphPtr_t& graph) {
  WeighedDistance* ptr = new WeighedDistance(robot, graph);
  WeighedDistancePtr_t shPtr(ptr);
  ptr->init(shPtr);
  return shPtr;
}

WeighedDistancePtr_t WeighedDistance::createCopy(
    const WeighedDistancePtr_t& distance) {
  WeighedDistance* ptr = new WeighedDistance(*distance);
  WeighedDistancePtr_t shPtr(ptr);
  ptr->init(shPtr);
  return shPtr;
}

core::DistancePtr_t WeighedDistance::clone() const {
  return createCopy(weak_.lock());
}

WeighedDistance::WeighedDistance(const DevicePtr_t& robot,
                                 const graph::GraphPtr_t graph)
    : core::WeighedDistance(robot), graph_(graph) {}

WeighedDistance::WeighedDistance(const WeighedDistance& distance)
    : core::WeighedDistance(distance), graph_(distance.graph_) {}

void WeighedDistance::init(WeighedDistanceWkPtr_t self) { weak_ = self; }

value_type WeighedDistance::impl_distance(ConfigurationIn_t q1,
                                          ConfigurationIn_t q2) const {
  value_type d = core::WeighedDistance::impl_distance(q1, q2);
  return d;

  // graph::Edges_t pes = graph_->getEdges
  // (graph_->getNode (q1), graph_->getNode (q2));
  // while (!pes.empty ()) {
  // if (pes.back ()->canConnect (q1, q2))
  // return d;
  // pes.pop_back ();
  // }
  // return d + 100;
}

value_type WeighedDistance::impl_distance(core::NodePtr_t n1,
                                          core::NodePtr_t n2) const {
  const Configuration_t &q1 = n1->configuration(), q2 = n2->configuration();
  value_type d = core::WeighedDistance::impl_distance(q1, q2);

  graph::Edges_t pes =
      graph_->getEdges(graph_->getState(static_cast<RoadmapNodePtr_t>(n1)),
                       graph_->getState(static_cast<RoadmapNodePtr_t>(n2)));
  while (!pes.empty()) {
    if (pes.back()->canConnect(q1, q2)) return d;
    pes.pop_back();
  }
  return d + 100;
}

template <typename Archive>
inline void WeighedDistance::serialize(Archive& ar,
                                       const unsigned int version) {
  using namespace boost::serialization;
  (void)version;
  ar& make_nvp("base", base_object<core::WeighedDistance>(*this));
  ar& BOOST_SERIALIZATION_NVP(graph_);
  ar& BOOST_SERIALIZATION_NVP(weak_);
}

HPP_SERIALIZATION_IMPLEMENT(WeighedDistance);

}  //   namespace manipulation
}  // namespace hpp

BOOST_CLASS_EXPORT(hpp::manipulation::WeighedDistance)
