// Copyright (c) 2015 CNRS
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

#ifndef HPP_MANIPULATION_DISTANCE_HH
#define HPP_MANIPULATION_DISTANCE_HH

#include <hpp/core/weighed-distance.hh>
#include <hpp/manipulation/config.hh>
#include <hpp/manipulation/fwd.hh>
#include <hpp/manipulation/graph/fwd.hh>

namespace hpp {
namespace manipulation {
/// \addtogroup steering_method
/// \{

/// Class for distance between configurations
class HPP_MANIPULATION_DLLAPI WeighedDistance : public core::WeighedDistance {
 public:
  static WeighedDistancePtr_t create(const DevicePtr_t& robot,
                                     const graph::GraphPtr_t& graph);

  static WeighedDistancePtr_t createCopy(const WeighedDistancePtr_t& distance);

  virtual core::DistancePtr_t clone() const;

  /// Set the graph of constraints
  void constraintGraph(const graph::GraphPtr_t& graph) { graph_ = graph; }

  /// Get the graph of constraints
  graph::GraphPtr_t constraintGraph() const { return graph_; }

 protected:
  WeighedDistance(const DevicePtr_t& robot, const graph::GraphPtr_t graph);

  WeighedDistance(const WeighedDistance& distance);

  /// Derived class should implement this function
  virtual value_type impl_distance(ConfigurationIn_t q1,
                                   ConfigurationIn_t q2) const;
  virtual value_type impl_distance(core::NodePtr_t n1,
                                   core::NodePtr_t n2) const;

  void init(WeighedDistanceWkPtr_t self);

 private:
  graph::GraphPtr_t graph_;
  WeighedDistanceWkPtr_t weak_;

  WeighedDistance() {}
  HPP_SERIALIZABLE();
};  // class Distance
/// \}
}  //   namespace manipulation
}  // namespace hpp
#endif  // HPP_MANIPULATION_DISTANCE_HH
