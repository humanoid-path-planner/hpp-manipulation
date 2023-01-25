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

#ifndef HPP_MANIPULATION_ROADMAP_HH
#define HPP_MANIPULATION_ROADMAP_HH

#include <hpp/core/roadmap.hh>
#include <hpp/manipulation/deprecated.hh>
#include <hpp/manipulation/leaf-connected-comp.hh>

#include "hpp/manipulation/config.hh"
#include "hpp/manipulation/fwd.hh"
#include "hpp/manipulation/graph/fwd.hh"

namespace hpp {
namespace manipulation {
/// \addtogroup roadmap
/// \{

/// Extension of hpp::core::Roadmap. It adds the ability of doing
/// statistics on the graph
class HPP_MANIPULATION_DLLAPI Roadmap : public core::Roadmap {
 public:
  typedef core::Roadmap Parent;

  /// Return a shared pointer to a new instance
  static RoadmapPtr_t create(const core::DistancePtr_t& distance,
                             const core::DevicePtr_t& robot);

  /// Register histogram so that each time a node is added to the roadmap,
  /// it is also added to the histogram
  void insertHistogram(const graph::HistogramPtr_t hist);

  /// Register the constraint graph to do statistics.
  void constraintGraph(const graph::GraphPtr_t& graph);

  /// Clear the histograms and call parent implementation.
  void clear();

  /// Catch event 'New node added'
  void push_node(const core::NodePtr_t& n);

  /// Get the nearest neighbor in a given graph::Node and in a given
  /// ConnectedComponent.
  RoadmapNodePtr_t nearestNodeInState(
      const ConfigurationPtr_t& configuration,
      const ConnectedComponentPtr_t& connectedComponent,
      const graph::StatePtr_t& state, value_type& minDistance) const;

  /// Get graph state corresponding to given roadmap node
  /// \deprecated use getState instead
  HPP_MANIPULATION_DEPRECATED graph::StatePtr_t getNode(RoadmapNodePtr_t node);

  /// Update the graph of connected components after new connection
  /// \param cc1, cc2 the two connected components that have just been
  /// connected.
  void connect(const LeafConnectedCompPtr_t& cc1,
               const LeafConnectedCompPtr_t& cc2);

  /// Merge two connected components
  /// \param cc1 the connected component to merge into
  /// \param the connected components to merge into cc1.
  void merge(const LeafConnectedCompPtr_t& cc1,
             LeafConnectedComp::LeafConnectedComps_t& ccs);

  /// Get graph state corresponding to given roadmap node
  graph::StatePtr_t getState(RoadmapNodePtr_t node);

  /// Get leaf connected components
  ///
  /// Leaf connected components are composed of nodes
  /// \li belonging to the same connected component of the roadmap and,
  /// \li lying in the same leaf of a transition.
  const LeafConnectedComps_t& leafConnectedComponents() const {
    return leafCCs_;
  }

 protected:
  /// Register a new configuration.
  void statInsert(const RoadmapNodePtr_t& n);

  /// Constructor
  Roadmap(const core::DistancePtr_t& distance, const core::DevicePtr_t& robot);

  /// Node factory
  core::NodePtr_t createNode(const ConfigurationPtr_t& config) const;

  void init(const RoadmapPtr_t& shPtr) {
    Parent::init(shPtr);
    weak_ = shPtr;
  }

  virtual void impl_addEdge(const core::EdgePtr_t& edge);

 private:
  typedef graph::Histograms_t Histograms_t;
  /// Keep track of the leaf that are explored.
  /// There should be one histogram per foliation.
  Histograms_t histograms_;
  graph::GraphPtr_t graph_;
  RoadmapWkPtr_t weak_;
  LeafConnectedComps_t leafCCs_;

  Roadmap() {}
  HPP_SERIALIZABLE();
};
/// \}
}  // namespace manipulation
}  // namespace hpp

#endif  // HPP_MANIPULATION_ROADMAP_HH
