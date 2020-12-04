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

# include <hpp/manipulation/fwd.hh>

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
inline void serialize(Archive & ar, hpp::manipulation::graph::EdgePtr_t& e, const unsigned int /*version*/)
{
  using hpp::serialization::archive_graph_wrapper;
  using namespace hpp::manipulation::graph;

  std::size_t id;
  if (Archive::is_saving::value) id = e->id();
  ar & BOOST_SERIALIZATION_NVP(id);
  if (!Archive::is_saving::value) {
    archive_graph_wrapper* agw = dynamic_cast<archive_graph_wrapper*>(&ar);
    if (agw == NULL)
      throw std::runtime_error("Cannot deserialize edges with a archive_graph_wrapper");
    GraphComponentPtr_t gc = agw->graph->get(id).lock();
    e = HPP_DYNAMIC_PTR_CAST(Edge, gc);
  }
}
} // namespace serialization
} // namespace boost

#endif // HPP_MANIPULATION_SERIALIZATION_HH
