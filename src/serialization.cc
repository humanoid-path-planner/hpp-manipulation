//
// Copyright (c) 2020 CNRS
// Authors: Joseph Mirabel
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
#include <boost/serialization/set.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/weak_ptr.hpp>

#include <pinocchio/serialization/eigen.hpp>
#include <hpp/util/serialization.hh>

#include <hpp/manipulation/device.hh>
#include <hpp/manipulation/leaf-connected-comp.hh>
#include <hpp/manipulation/roadmap-node.hh>
#include <hpp/manipulation/roadmap.hh>
#include <hpp/manipulation/serialization.hh>

BOOST_CLASS_EXPORT_IMPLEMENT(hpp::manipulation::RoadmapNode)
BOOST_CLASS_EXPORT_IMPLEMENT(hpp::manipulation::ConnectedComponent)
BOOST_CLASS_EXPORT_IMPLEMENT(hpp::manipulation::WeighedLeafConnectedComp)
BOOST_CLASS_EXPORT_IMPLEMENT(hpp::manipulation::Roadmap)

namespace hpp {
namespace manipulation {

template <typename Archive>
inline void RoadmapNode::serialize(Archive& ar, const unsigned int version)
{
  using namespace boost::serialization;
  (void) version;
  ar & make_nvp("base", base_object<core::Node>(*this));
  ar & BOOST_SERIALIZATION_NVP(state_);
  ar & BOOST_SERIALIZATION_NVP(leafCC_);
}
HPP_SERIALIZATION_IMPLEMENT(RoadmapNode);

template <typename Archive>
inline void ConnectedComponent::serialize(Archive& ar, const unsigned int version)
{
  using namespace boost::serialization;
  (void) version;
  ar & make_nvp("base", base_object<core::ConnectedComponent>(*this));
  ar & BOOST_SERIALIZATION_NVP(roadmap_);
  if (!Archive::is_saving::value) {
    RoadmapPtr_t roadmap = roadmap_.lock();
    for (const core::NodePtr_t& node : nodes()) {
      const RoadmapNodePtr_t& n = static_cast <const RoadmapNodePtr_t> (node);
      graphStateMap_[roadmap->getState(n)].push_back(n);
    }
  }
  //ar & BOOST_SERIALIZATION_NVP(graphStateMap_);
}
HPP_SERIALIZATION_IMPLEMENT(ConnectedComponent);

template <typename Archive>
inline void LeafConnectedComp::serialize(Archive& ar, const unsigned int version)
{
  (void) version;
  ar & BOOST_SERIALIZATION_NVP(state_);
  ar & BOOST_SERIALIZATION_NVP(nodes_);

  //ar & BOOST_SERIALIZATION_NVP(explored_);
  ar & BOOST_SERIALIZATION_NVP(roadmap_);
  ar & BOOST_SERIALIZATION_NVP(to_);
  ar & BOOST_SERIALIZATION_NVP(from_);
  ar & BOOST_SERIALIZATION_NVP(weak_);
}
HPP_SERIALIZATION_IMPLEMENT(LeafConnectedComp);

template <typename Archive>
inline void WeighedLeafConnectedComp::serialize(Archive& ar, const unsigned int version)
{
  using namespace boost::serialization;
  (void) version;
  ar & make_nvp("base", base_object<LeafConnectedComp>(*this));
  ar & BOOST_SERIALIZATION_NVP(weight_);
  ar & BOOST_SERIALIZATION_NVP(p_);
  ar & BOOST_SERIALIZATION_NVP(edges_);
}
HPP_SERIALIZATION_IMPLEMENT(WeighedLeafConnectedComp);

template <typename Archive>
inline void Roadmap::serialize(Archive& ar, const unsigned int version)
{
  using namespace boost::serialization;
  (void) version;
  // Must deserialize the graph before the connected components (so the base class).
  ar & BOOST_SERIALIZATION_NVP(graph_);
  ar & BOOST_SERIALIZATION_NVP(weak_);
  ar & make_nvp("base", base_object<core::Roadmap>(*this));
  //ar & BOOST_SERIALIZATION_NVP(histograms_);
  ar & BOOST_SERIALIZATION_NVP(leafCCs_);
}
HPP_SERIALIZATION_IMPLEMENT(Roadmap);

} //   namespace core
} // namespace hpp
