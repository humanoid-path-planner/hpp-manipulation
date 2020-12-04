//
// Copyright (c) 2020 CNRS
// Author: Joseph Mirabel
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

#ifndef HPP_MANIPULATION_SERIALIZATION_HH
# define HPP_MANIPULATION_SERIALIZATION_HH

# include <boost/serialization/split_free.hpp>
# include <boost/serialization/shared_ptr.hpp>
# include <boost/serialization/weak_ptr.hpp>

# include <hpp/util/serialization.hh>
# include <hpp/pinocchio/serialization.hh>

# include <hpp/manipulation/fwd.hh>

# include <hpp/manipulation/graph/graph.hh>
# include <hpp/manipulation/graph/state.hh>
# include <hpp/manipulation/graph/edge.hh>

namespace hpp {
namespace serialization {
struct archive_graph_wrapper {
  manipulation::graph::GraphPtr_t graph;
  virtual ~archive_graph_wrapper() {}
};
} // namespace manipulation
} // namespace hpp

namespace boost {
namespace serialization {
template<class Archive>
inline void serialize(Archive & ar, hpp::manipulation::graph::GraphPtr_t& g, const unsigned int version)
{
  using hpp::serialization::archive_graph_wrapper;
  using namespace hpp::manipulation::graph;
  (void) version;

  std::size_t id;
  if (Archive::is_saving::value) id = g->id();
  ar & BOOST_SERIALIZATION_NVP(id);
  if (!Archive::is_saving::value) {
    archive_graph_wrapper* agw = dynamic_cast<archive_graph_wrapper*>(&ar);
    if (agw == NULL)
      throw std::runtime_error("Cannot deserialize edges with a archive_graph_wrapper");
    g = agw->graph;
  }
}

template<class Archive>
inline void serialize(Archive & ar, hpp::manipulation::graph::EdgePtr_t& e, const unsigned int version)
{
  using hpp::serialization::archive_graph_wrapper;
  using namespace hpp::manipulation::graph;
  (void) version;

  std::size_t id;
  if (Archive::is_saving::value) id = (e ? e->id() : -1);
  ar & BOOST_SERIALIZATION_NVP(id);
  if (!Archive::is_saving::value) {
    archive_graph_wrapper* agw = dynamic_cast<archive_graph_wrapper*>(&ar);
    if (agw == NULL)
      throw std::runtime_error("Cannot deserialize edges with a archive_graph_wrapper");
    GraphComponentPtr_t gc = agw->graph->get(id).lock();
    e = HPP_DYNAMIC_PTR_CAST(Edge, gc);
  }
}

template<class Archive>
inline void serialize(Archive & ar, hpp::manipulation::graph::StatePtr_t& s, const unsigned int version)
{
  using hpp::serialization::archive_graph_wrapper;
  using namespace hpp::manipulation::graph;
  (void) version;

  std::size_t id;
  if (Archive::is_saving::value) id = (s ? s->id() : -1);
  ar & BOOST_SERIALIZATION_NVP(id);
  if (!Archive::is_saving::value) {
    archive_graph_wrapper* agw = dynamic_cast<archive_graph_wrapper*>(&ar);
    if (agw == NULL)
      throw std::runtime_error("Cannot deserialize edges with a archive_graph_wrapper");
    GraphComponentPtr_t gc = agw->graph->get(id).lock();
    s = HPP_DYNAMIC_PTR_CAST(State, gc);
  }
}

template<class Archive>
inline void serialize(Archive & ar, hpp::manipulation::graph::EdgeWkPtr_t& e, const unsigned int version)
{
  using namespace hpp::manipulation::graph;
  EdgePtr_t e_ = e.lock();
  serialize(ar, e_, version);
  e = e_;
}

template<class Archive>
inline void serialize(Archive & ar, hpp::manipulation::graph::StateWkPtr_t& s, const unsigned int version)
{
  using namespace hpp::manipulation::graph;
  StatePtr_t s_ = s.lock();
  serialize(ar, s_, version);
  s = s_;
}

template<class Archive>
inline void load (Archive& ar, hpp::manipulation::DevicePtr_t& d, const unsigned int version)
{
  load<Archive, hpp::manipulation::Device> (ar, d, version);
  using hpp::serialization::archive_device_wrapper;
  archive_device_wrapper* adw = dynamic_cast<archive_device_wrapper*>(&ar);
  if (adw) d = boost::dynamic_pointer_cast<hpp::manipulation::Device>(adw->device);
}

template<class Archive>
inline void load (Archive& ar, hpp::manipulation::DeviceWkPtr_t& d, const unsigned int version)
{
  load<Archive, hpp::manipulation::Device> (ar, d, version);
  using hpp::serialization::archive_device_wrapper;
  archive_device_wrapper* adw = dynamic_cast<archive_device_wrapper*>(&ar);
  if (adw) d = boost::dynamic_pointer_cast<hpp::manipulation::Device>(adw->device);
}
} // namespace serialization
} // namespace boost

#endif // HPP_MANIPULATION_SERIALIZATION_HH
