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

#ifndef HPP_MANIPULATION_GRAPH_FWD_HH
#define HPP_MANIPULATION_GRAPH_FWD_HH

#include <hpp/statistics/distribution.hh>
#include <hpp/util/pointer.hh>
#include <vector>

namespace hpp {
namespace manipulation {
namespace graph {
HPP_PREDEF_CLASS(Graph);
HPP_PREDEF_CLASS(Edge);
HPP_PREDEF_CLASS(State);
HPP_PREDEF_CLASS(WaypointEdge);
HPP_PREDEF_CLASS(LevelSetEdge);
HPP_PREDEF_CLASS(StateSelector);
HPP_PREDEF_CLASS(GraphComponent);
HPP_PREDEF_CLASS(GuidedStateSelector);
typedef shared_ptr<Graph> GraphPtr_t;
typedef shared_ptr<State> StatePtr_t;
typedef shared_ptr<Edge> EdgePtr_t;
typedef shared_ptr<WaypointEdge> WaypointEdgePtr_t;
typedef shared_ptr<LevelSetEdge> LevelSetEdgePtr_t;
typedef shared_ptr<StateSelector> StateSelectorPtr_t;
typedef shared_ptr<GuidedStateSelector> GuidedStateSelectorPtr_t;
typedef shared_ptr<GraphComponent> GraphComponentPtr_t;
typedef std::vector<GraphComponentWkPtr_t> GraphComponents_t;
typedef std::vector<StatePtr_t> States_t;
typedef std::vector<EdgePtr_t> Edges_t;
typedef ::hpp::statistics::DiscreteDistribution<EdgePtr_t>::Weight_t Weight_t;
typedef ::hpp::statistics::DiscreteDistribution<EdgePtr_t> Neighbors_t;
typedef std::vector<StateSelectorPtr_t> StateSelectors_t;

typedef hpp::core::segments_t segments_t;
typedef std::vector<segments_t> IntervalsContainer_t;
typedef hpp::core::NumericalConstraints_t NumericalConstraints_t;
typedef hpp::core::LockedJoints_t LockedJoints_t;

class Histogram;
class StateHistogram;
class LeafHistogram;
typedef shared_ptr<Histogram> HistogramPtr_t;
typedef shared_ptr<StateHistogram> StateHistogramPtr_t;
typedef shared_ptr<LeafHistogram> LeafHistogramPtr_t;
typedef std::list<HistogramPtr_t> Histograms_t;

class Validation;
typedef shared_ptr<Validation> ValidationPtr_t;
}  // namespace graph
}  // namespace manipulation
}  // namespace hpp

#endif  // HPP_MANIPULATION_GRAPH_FWD_HH
