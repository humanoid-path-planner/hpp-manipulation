//
// Copyright (c) 2020 CNRS
// Author: Joseph Mirabel
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

#ifndef HPP_MANIPULATION_SERIALIZATION_HH
#define HPP_MANIPULATION_SERIALIZATION_HH

#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/split_free.hpp>
#include <boost/serialization/weak_ptr.hpp>
#include <hpp/manipulation/fwd.hh>
#include <hpp/manipulation/graph/edge.hh>
#include <hpp/manipulation/graph/graph.hh>
#include <hpp/manipulation/graph/state.hh>
#include <hpp/pinocchio/serialization.hh>
#include <hpp/util/serialization.hh>

namespace hpp {
namespace serialization {
template <typename Archive>
manipulation::graph::GraphPtr_t getGraphFromArchive(Archive& ar,
                                                    const std::string& name) {
  auto* har = hpp::serialization::cast(&ar);
  if (!har || !har->contains(name))
    throw std::runtime_error(
        "Cannot deserialize edges with a provided graph with correct name.");
  return har->template get<manipulation::graph::Graph>(name, true)->self();
}

template <class Archive, class GraphCompT>
inline void serializeGraphComponent(Archive& ar, shared_ptr<GraphCompT>& c,
                                    const unsigned int version) {
  (void)version;

  std::size_t id;
  std::string name;
  if (Archive::is_saving::value) {
    id = (c ? c->id() : -1);
    if (c && c->parentGraph()) name = c->parentGraph()->name();
  }
  ar& BOOST_SERIALIZATION_NVP(id);
  ar& BOOST_SERIALIZATION_NVP(name);
  if (!Archive::is_saving::value) {
    auto graph = getGraphFromArchive(ar, name);
    c = HPP_DYNAMIC_PTR_CAST(GraphCompT, graph->get(id).lock());
  }
}
}  // namespace serialization
}  // namespace hpp

BOOST_CLASS_EXPORT_KEY(hpp::manipulation::RoadmapNode)
BOOST_CLASS_EXPORT_KEY(hpp::manipulation::ConnectedComponent)
BOOST_CLASS_EXPORT_KEY(hpp::manipulation::WeighedLeafConnectedComp)
BOOST_CLASS_EXPORT_KEY(hpp::manipulation::Roadmap)

namespace boost {
namespace serialization {
template <class Archive>
inline void serialize(Archive& ar, hpp::manipulation::graph::GraphPtr_t& g,
                      const unsigned int version) {
  using hpp::serialization::getGraphFromArchive;
  (void)version;

  std::string name;
  if (Archive::is_saving::value) name = g->name();
  ar& BOOST_SERIALIZATION_NVP(name);
  if (!Archive::is_saving::value) g = getGraphFromArchive(ar, name);
}

template <class Archive>
inline void serialize(Archive& ar, hpp::manipulation::graph::EdgePtr_t& e,
                      const unsigned int version) {
  hpp::serialization::serializeGraphComponent(ar, e, version);
}

template <class Archive>
inline void serialize(Archive& ar, hpp::manipulation::graph::StatePtr_t& s,
                      const unsigned int version) {
  hpp::serialization::serializeGraphComponent(ar, s, version);
}

template <class Archive>
inline void serialize(Archive& ar, hpp::manipulation::graph::EdgeWkPtr_t& e,
                      const unsigned int version) {
  auto e_ = e.lock();
  serialize(ar, e_, version);
  e = e_;
}

template <class Archive>
inline void serialize(Archive& ar, hpp::manipulation::graph::StateWkPtr_t& s,
                      const unsigned int version) {
  auto s_ = s.lock();
  serialize(ar, s_, version);
  s = s_;
}

template <class Archive>
inline void load(Archive& ar, hpp::manipulation::DevicePtr_t& d,
                 const unsigned int version) {
  load<Archive, hpp::manipulation::Device>(ar, d, version);
  auto* har = hpp::serialization::cast(&ar);
  if (d && har && har->contains(d->name()))
    d = har->template getChildClass<hpp::pinocchio::Device,
                                    hpp::manipulation::Device>(d->name(), true)
            ->self();
}

template <class Archive>
inline void load(Archive& ar, hpp::manipulation::DeviceWkPtr_t& d,
                 const unsigned int version) {
  load<Archive, hpp::manipulation::Device>(ar, d, version);
  auto* har = hpp::serialization::cast(&ar);
  auto dd = d.lock();
  if (!dd) return;
  if (har && har->contains(dd->name()))
    d = har->template getChildClass<hpp::pinocchio::Device,
                                    hpp::manipulation::Device>(dd->name(), true)
            ->self();
}
}  // namespace serialization
}  // namespace boost

#endif  // HPP_MANIPULATION_SERIALIZATION_HH
