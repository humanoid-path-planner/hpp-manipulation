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

#include "hpp/manipulation/constraint-set.hh"

#include <hpp/util/serialization.hh>

#include "hpp/manipulation/device.hh"
#include "hpp/manipulation/graph/edge.hh"
#include "hpp/manipulation/serialization.hh"

namespace hpp {
namespace manipulation {
ConstraintSetPtr_t ConstraintSet::create(const DevicePtr_t& robot,
                                         const std::string& name) {
  ConstraintSet* ptr = new ConstraintSet(robot, name);
  ConstraintSetPtr_t shPtr(ptr);
  ptr->init(shPtr);
  return shPtr;
}

ConstraintSetPtr_t ConstraintSet::createCopy(const ConstraintSetPtr_t& cs) {
  ConstraintSet* ptr = new ConstraintSet(*cs);
  ConstraintSetPtr_t shPtr(ptr);
  ptr->init(shPtr);
  return shPtr;
}

ConstraintPtr_t ConstraintSet::copy() const { return createCopy(weak_.lock()); }

void ConstraintSet::edge(graph::EdgePtr_t edge) { edge_ = edge; }

graph::EdgePtr_t ConstraintSet::edge() const { return edge_; }

ConstraintSet::ConstraintSet(const DevicePtr_t& robot, const std::string& name)
    : Parent_t(robot, name), edge_() {}

ConstraintSet::ConstraintSet(const ConstraintSet& other)
    : Parent_t(other), edge_(other.edge()) {}

void ConstraintSet::init(const ConstraintSetPtr_t& self) {
  Parent_t::init(self);
  weak_ = self;
}

std::ostream& ConstraintSet::print(std::ostream& os) const {
  Parent_t::print(os);
  if (edge_) os << iendl << "Built by edge " << edge_->name();
  return os;
}

template <class Archive>
void ConstraintSet::serialize(Archive& ar, const unsigned int version) {
  using namespace boost::serialization;
  (void)version;
  ar& make_nvp("base", base_object<core::ConstraintSet>(*this));
  ar& BOOST_SERIALIZATION_NVP(edge_);
  ar& BOOST_SERIALIZATION_NVP(weak_);
}

HPP_SERIALIZATION_IMPLEMENT(ConstraintSet);
}  // namespace manipulation
}  // namespace hpp

BOOST_CLASS_EXPORT_IMPLEMENT(hpp::manipulation::ConstraintSet)
