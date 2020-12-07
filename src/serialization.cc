//
// Copyright (c) 2020 CNRS
// Authors: Joseph Mirabel
//
// This file is part of hpp-manipulation
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
// hpp-manipulation  If not, see
// <http://www.gnu.org/licenses/>.

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/weak_ptr.hpp>

#include <pinocchio/serialization/eigen.hpp>
#include <hpp/util/serialization.hh>

#include <hpp/manipulation/leaf-connected-comp.hh>
#include <hpp/manipulation/roadmap-node.hh>
#include <hpp/manipulation/roadmap.hh>
#include <hpp/manipulation/serialization.hh>

BOOST_CLASS_EXPORT_IMPLEMENT(hpp::manipulation::RoadmapNode)
BOOST_CLASS_EXPORT_IMPLEMENT(hpp::manipulation::LeafConnectedComp)
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
  ar & BOOST_SERIALIZATION_NVP(cacheSystem_);
  ar & BOOST_SERIALIZATION_NVP(state_);
  //ar & BOOST_SERIALIZATION_NVP(leafCC_);
}
HPP_SERIALIZATION_IMPLEMENT(RoadmapNode);

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
  ar & make_nvp("base", base_object<core::Roadmap>(*this));
  //ar & BOOST_SERIALIZATION_NVP(histograms_);
  ar & BOOST_SERIALIZATION_NVP(graph_);
  ar & BOOST_SERIALIZATION_NVP(leafCCs_);
  ar & BOOST_SERIALIZATION_NVP(weak_);
}
HPP_SERIALIZATION_IMPLEMENT(Roadmap);

} //   namespace core
} // namespace hpp
