// Copyright (c) 2014, LAAS-CNRS
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//
// This file is part of hpp-manipulation.
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
// hpp-manipulation. If not, see <http://www.gnu.org/licenses/>.

#ifndef HPP_MANIPULATION_GRAPH_FWD_HH
# define HPP_MANIPULATION_GRAPH_FWD_HH

#include <hpp/util/pointer.hh>
#include <hpp/statistics/distribution.hh>
#include <vector>

namespace hpp {
  namespace manipulation {
    namespace graph {
      HPP_PREDEF_CLASS (Graph);
      HPP_PREDEF_CLASS (Edge);
      HPP_PREDEF_CLASS (State);
      HPP_PREDEF_CLASS (WaypointEdge);
      HPP_PREDEF_CLASS (LevelSetEdge);
      HPP_PREDEF_CLASS (StateSelector);
      HPP_PREDEF_CLASS (GraphComponent);
      HPP_PREDEF_CLASS (GuidedStateSelector);
      typedef boost::shared_ptr < Graph > GraphPtr_t;
      typedef boost::shared_ptr < State > StatePtr_t;
      typedef boost::shared_ptr < Edge > EdgePtr_t;
      typedef boost::shared_ptr < WaypointEdge > WaypointEdgePtr_t;
      typedef boost::shared_ptr < LevelSetEdge > LevelSetEdgePtr_t;
      typedef boost::shared_ptr < StateSelector > StateSelectorPtr_t;
      typedef boost::shared_ptr < GuidedStateSelector >
      GuidedStateSelectorPtr_t;
      typedef boost::shared_ptr < GraphComponent > GraphComponentPtr_t;
      typedef std::vector < GraphComponentWkPtr_t > GraphComponents_t;
      typedef std::vector < StatePtr_t > States_t;
      typedef std::vector < EdgePtr_t > Edges_t;
      typedef ::hpp::statistics::DiscreteDistribution< EdgePtr_t >::Weight_t Weight_t;
      typedef ::hpp::statistics::DiscreteDistribution< EdgePtr_t > Neighbors_t;
      typedef std::vector < StateSelectorPtr_t > StateSelectors_t;

      typedef hpp::core::segments_t segments_t;
      typedef std::vector <segments_t> IntervalsContainer_t;
      typedef hpp::core::NumericalConstraints_t NumericalConstraints_t;
      typedef hpp::core::LockedJoints_t LockedJoints_t;

      class Histogram;
      class StateHistogram;
      class LeafHistogram;
      typedef boost::shared_ptr <Histogram> HistogramPtr_t;
      typedef boost::shared_ptr <StateHistogram> StateHistogramPtr_t;
      typedef boost::shared_ptr <LeafHistogram> LeafHistogramPtr_t;
      typedef std::list < HistogramPtr_t > Histograms_t;
    } // namespace graph
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_GRAPH_FWD_HH
