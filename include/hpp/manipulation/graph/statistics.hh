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

#ifndef HPP_MANIPULATION_GRAPH_STATISTICS_HH
#define HPP_MANIPULATION_GRAPH_STATISTICS_HH

#include <hpp/manipulation/roadmap-node.hh>
#include <hpp/statistics/bin.hh>
#include <hpp/util/debug.hh>

#include "hpp/manipulation/config.hh"
#include "hpp/manipulation/fwd.hh"
#include "hpp/manipulation/graph/graph.hh"
#include "hpp/manipulation/graph/state.hh"

namespace hpp {
namespace manipulation {
namespace graph {
/// This class is used to do statistics on the roadmap.
/// It keeps a track of which leaves of a foliation have been visited.
class HPP_MANIPULATION_DLLAPI LeafBin : public ::hpp::statistics::Bin {
 public:
  typedef ::hpp::statistics::Bin Parent;
  typedef std::list<RoadmapNodePtr_t> RoadmapNodes_t;

  LeafBin(const vector_t& v, value_type* threshold_);

  void push_back(const RoadmapNodePtr_t& n);

  bool operator<(const LeafBin& rhs) const;

  bool operator==(const LeafBin& rhs) const;

  const vector_t& value() const;

  std::ostream& print(std::ostream& os) const;

  unsigned int numberOfObsOutOfConnectedComponent(
      const core::ConnectedComponentPtr_t& cc) const;

  const RoadmapNodes_t& nodes() const;

 private:
  vector_t value_;

  RoadmapNodes_t nodes_;

  value_type* thr_;

  std::ostream& printValue(std::ostream& os) const;
};

/// This class is used to do statistics on the roadmap.
/// It keeps a track of which nodes of the constraint graph have been visited.
class HPP_MANIPULATION_DLLLOCAL NodeBin : public ::hpp::statistics::Bin {
 public:
  typedef ::hpp::statistics::Bin Parent;
  NodeBin(const StatePtr_t& n);

  void push_back(const RoadmapNodePtr_t& n);

  bool operator<(const NodeBin& rhs) const;

  bool operator==(const NodeBin& rhs) const;

  const StatePtr_t& state() const;

  std::ostream& print(std::ostream& os) const;

 private:
  StatePtr_t state_;

  typedef std::list<RoadmapNodePtr_t> RoadmapNodes_t;
  RoadmapNodes_t roadmapNodes_;

  std::ostream& printValue(std::ostream& os) const;
};

class HPP_MANIPULATION_DLLLOCAL Histogram {
 public:
  virtual ~Histogram(){};

  virtual void add(const RoadmapNodePtr_t& node) = 0;

  virtual HistogramPtr_t clone() const = 0;

  virtual void clear() = 0;
};

/// This class represents a foliation of a submanifold of the configuration
/// space.
/// Such a foliation is defined by the two following functions:
/// \li a condition $f$ such that the submanifold is
///                     $\mathcal{M}= \left{q \in \mathbb{C} | f(q)=0 \right}$
/// \li a parametrizer $g$ such that the leaf of this foliation is a level
///      set of $g$.
class HPP_MANIPULATION_DLLAPI Foliation {
 public:
  /// Whether the configuration is the submanifold $\mathcal{M}$
  bool contains(ConfigurationIn_t q) const;
  /// Whether the configuration is the submanifold $\mathcal{M}$
  vector_t parameter(ConfigurationIn_t q) const;

  void condition(const ConstraintSetPtr_t c);
  ConstraintSetPtr_t condition() const;
  void parametrizer(const ConstraintSetPtr_t p);
  ConstraintSetPtr_t parametrizer() const;

 private:
  // condition_ contains the constraints defining the submanifold
  // containing all the leaf.
  // parametrizer_ contains the constraints providing a parametrization
  // of the foliation.
  ConstraintSetPtr_t condition_, parametrizer_;
};

class HPP_MANIPULATION_DLLAPI LeafHistogram
    : public ::hpp::statistics::Statistics<LeafBin>,
      public Histogram {
 public:
  typedef ::hpp::statistics::Statistics<LeafBin> Parent;

  static LeafHistogramPtr_t create(const Foliation f);

  /// Insert an occurence of a value in the histogram
  void add(const RoadmapNodePtr_t& n);

  std::ostream& print(std::ostream& os) const;

  virtual HistogramPtr_t clone() const;

  statistics::DiscreteDistribution<RoadmapNodePtr_t>
  getDistribOutOfConnectedComponent(
      const core::ConnectedComponentPtr_t& cc) const;

  statistics::DiscreteDistribution<RoadmapNodePtr_t> getDistrib() const;

  void clear() { Parent::clear(); }

  const Foliation& foliation() const { return f_; }

 protected:
  /// Constructor
  /// \param state defines the submanifold containing the foliation.
  /// \param constraint The constraint that create the foliation being
  ///        studied.
  LeafHistogram(const Foliation f);

 private:
  Foliation f_;

  /// Threshold used for equality between offset values.
  value_type threshold_;
};

class HPP_MANIPULATION_DLLLOCAL StateHistogram
    : public ::hpp::statistics::Statistics<NodeBin>,
      public Histogram {
 public:
  typedef ::hpp::statistics::Statistics<NodeBin> Parent;
  /// Constructor
  /// \param graph The constraint graph used to get the states from
  ///        a configuration.
  StateHistogram(const graph::GraphPtr_t& graph);

  /// Insert an occurence of a value in the histogram
  void add(const RoadmapNodePtr_t& n);

  std::ostream& print(std::ostream& os) const;

  const graph::GraphPtr_t& constraintGraph() const;

  virtual HistogramPtr_t clone() const;

  void clear() { Parent::clear(); }

 private:
  /// The constraint graph
  graph::GraphPtr_t graph_;
};
typedef shared_ptr<StateHistogram> NodeHistogramPtr_t;
}  // namespace graph
}  // namespace manipulation
}  // namespace hpp

#endif  // HPP_MANIPULATION_GRAPH_STATISTICS_HH
