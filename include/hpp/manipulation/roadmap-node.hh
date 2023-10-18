// Copyright (c) 2014 CNRS
// Authors: Joseph Mirabel
//
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

#ifndef HPP_MANIPULATION_ROADMAP_NODE_HH
#define HPP_MANIPULATION_ROADMAP_NODE_HH

#include <hpp/core/node.hh>
#include <hpp/manipulation/config.hh>
#include <hpp/manipulation/connected-component.hh>
#include <hpp/manipulation/deprecated.hh>
#include <hpp/manipulation/fwd.hh>
#include <hpp/manipulation/graph/fwd.hh>

namespace hpp {
namespace manipulation {
class HPP_MANIPULATION_DLLAPI RoadmapNode : public core::Node {
 public:
  RoadmapNode(ConfigurationIn_t configuration)
      : core::Node(configuration), state_() {}

  RoadmapNode(ConfigurationIn_t configuration, ConnectedComponentPtr_t cc);

  /// \name Cache
  /// \{

  /// Get the caching system being used.
  bool cacheUpToDate() const { return static_cast<bool>(graphState()); }

  /// Getter for the graph::State.
  graph::StatePtr_t graphState() const { return state_.lock(); }

  /// Setter for the graph::State.
  void graphState(const graph::StatePtr_t& state) { state_ = state; }
  /// \}

  void leafConnectedComponent(const LeafConnectedCompPtr_t& sc) {
    leafCC_ = sc;
  }

  LeafConnectedCompPtr_t leafConnectedComponent() const { return leafCC_; }

 private:
  graph::StateWkPtr_t state_;
  LeafConnectedCompPtr_t leafCC_;

  RoadmapNode() {}
  HPP_SERIALIZABLE();
};
}  // namespace manipulation
}  // namespace hpp

#endif  // HPP_MANIPULATION_ROADMAP_NODE_HH
