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


#ifndef HPP_MANIPULATION_GRAPH_STATISTICS_HH
# define HPP_MANIPULATION_GRAPH_STATISTICS_HH

# include <hpp/util/debug.hh>

# include <hpp/core/node.hh>
# include <hpp/core/constraint-set.hh>
# include <hpp/statistics/bin.hh>

# include "hpp/manipulation/config.hh"
# include "hpp/manipulation/fwd.hh"
# include "hpp/manipulation/graph/graph.hh"
# include "hpp/manipulation/graph/node.hh"

namespace hpp {
  namespace manipulation {
    namespace graph {
      /// This class is used to do statistics on the roadmap.
      /// It keeps a track of which leaves of a foliation have been visited.
      class HPP_MANIPULATION_DLLAPI LeafBin : public ::hpp::statistics::Bin
      {
        public :
          typedef ::hpp::statistics::Bin Parent;
          typedef std::list <core::NodePtr_t> RoadmapNodes_t;

          LeafBin(const vector_t& v, value_type* threshold_);

          void push_back(const core::NodePtr_t& n);

          bool operator<(const LeafBin& rhs) const;

          bool operator==(const LeafBin& rhs) const;

          const vector_t& value () const;

          std::ostream& print (std::ostream& os) const;

          unsigned int numberOfObsOutOfConnectedComponent (const core::ConnectedComponentPtr_t& cc) const;

          const RoadmapNodes_t& nodes () const;

        private:
          vector_t value_;

          RoadmapNodes_t nodes_;

          value_type* thr_;

          std::ostream& printValue (std::ostream& os) const;
      };

      /// This class is used to do statistics on the roadmap.
      /// It keeps a track of which nodes of the constraint graph have been visited.
      class HPP_MANIPULATION_DLLLOCAL NodeBin : public ::hpp::statistics::Bin
      {
        public :
          typedef ::hpp::statistics::Bin Parent;
          NodeBin(const NodePtr_t& n);

          void push_back(const core::NodePtr_t& n);

          bool operator<(const NodeBin& rhs) const;

          bool operator==(const NodeBin& rhs) const;

          const NodePtr_t& node () const;

          std::ostream& print (std::ostream& os) const;

        private:
          NodePtr_t node_;

          typedef std::list <core::NodePtr_t> RoadmapNodes_t;
          RoadmapNodes_t roadmapNodes_;

          std::ostream& printValue (std::ostream& os) const;
      };

      class HPP_MANIPULATION_DLLLOCAL Histogram
      {
        public:
          virtual void add (const core::NodePtr_t& node) = 0;

          virtual HistogramPtr_t clone () const = 0;
      };

      class HPP_MANIPULATION_DLLLOCAL LeafHistogram : public ::hpp::statistics::Statistics < LeafBin >
                                                      , public Histogram
      {
        public:
          typedef ::hpp::statistics::Statistics < LeafBin > Parent;
          /// Constructor
          /// \param constraint The constraint that create the foliation being
          ///        studied.
          LeafHistogram (const ConstraintSetPtr_t& constraint);

          /// Insert an occurence of a value in the histogram
          void add (const core::NodePtr_t& n);

          std::ostream& print (std::ostream& os) const;

          const ConstraintSetPtr_t& constraint () const;

          virtual HistogramPtr_t clone () const;

          statistics::DiscreteDistribution < core::NodePtr_t > getDistribOutOfConnectedComponent (
              const core::ConnectedComponentPtr_t& cc) const;

        private:
          /// The constraint that creates the foliation.
          ConstraintSetPtr_t constraint_;

          /// Threshold used for equality between offset values.
          value_type threshold_;
      };

      class HPP_MANIPULATION_DLLLOCAL NodeHistogram : public ::hpp::statistics::Statistics < NodeBin >
                                                      , public Histogram
      {
        public:
          typedef ::hpp::statistics::Statistics < NodeBin > Parent;
          /// Constructor
          /// \param graph The constraint graph used to get the nodes from
          ///        a configuration.
          NodeHistogram (const graph::GraphPtr_t& graph);

          /// Insert an occurence of a value in the histogram
          void add (const core::NodePtr_t& n);

          std::ostream& print (std::ostream& os) const;

          const graph::GraphPtr_t& constraintGraph () const;

          virtual HistogramPtr_t clone () const;

        private:
          /// The constraint graph
          graph::GraphPtr_t graph_;
      };
    } // namespace graph
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_GRAPH_STATISTICS_HH
