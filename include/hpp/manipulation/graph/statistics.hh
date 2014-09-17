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
      /// It keeps a track of which leaf of a foliation have been visited.
      class HPP_MANIPULATION_DLLLOCAL LeafBin : public ::hpp::statistics::Bin
      {
        public :
          typedef ::hpp::statistics::Bin Parent;
          LeafBin(const vector_t& v): value_(v), nodes_() {}

          void push_back(const core::NodePtr_t& n) {
            nodes_.push_back(n);
          }

          bool operator<(const LeafBin& rhs) const {
            const vector_t& v = rhs.value ();
            assert (value_.size() == v.size());
            for (int p = 0; p < value_.size(); p++) {
              if (value_[p] != v[p])
                return value_[p] < v[p];
            }
            return false;
          }

          bool operator==(const LeafBin& rhs) const {
            return value_ == rhs.value();
          }

          const vector_t& value () const {
            return value_;
          }

          std::ostream& print (std::ostream& os) const
          {
            Parent::print (os) << " (";
            /// Sort by connected component.
            typedef std::list <RoadmapNodes_t> NodesList_t;
            NodesList_t l;
            bool found;
            for (RoadmapNodes_t::const_iterator itn = nodes_.begin ();
                itn != nodes_.end (); itn++) {
              found = false;
              for (NodesList_t::iterator itc = l.begin ();
                  itc != l.end (); itc++) {
                if ((*itn)->connectedComponent () == itc->front ()->connectedComponent ()) {
                  itc->push_back (*itn);
                  found = true;
                  break;
                }
              }
              if (!found) {
                l.push_back (RoadmapNodes_t (1, *itn));
              }
            }
            for (NodesList_t::iterator itc = l.begin ();
                itc != l.end (); itc++)
              os << itc->front ()->connectedComponent () << " - " << itc->size () << ", ";
            return os << ").";
          }

        private:
          vector_t value_;

          typedef std::list <core::NodePtr_t> RoadmapNodes_t;
          RoadmapNodes_t nodes_;

          std::ostream& printValue (std::ostream& os) const
          {
            os << "LeafBin (";
            size_t s = value_.size();
            if (s != 0) {
              for (size_t i = 0; i < s - 1; i++)
                os << value_[i] << "; ";
              os << value_[s-1];
            }
            os << ")";
            return os;
          }
      };

      /// This class is used to do statistics on the roadmap.
      /// It keeps a track of which leaf of a foliation have been visited.
      class HPP_MANIPULATION_DLLLOCAL NodeBin : public ::hpp::statistics::Bin
      {
        public :
          typedef ::hpp::statistics::Bin Parent;
          NodeBin(const Nodes_t& ns): nodes_(ns), roadmapNodes_() {}

          void push_back(const core::NodePtr_t& n) {
            roadmapNodes_.push_back(n);
          }

          bool operator<(const NodeBin& rhs) const {
            Nodes_t::const_iterator it1,
              it2 = rhs.nodes ().begin();
            for (it1 = nodes_.begin(); it1 != nodes_.end(); it1++) {
              if ((*it1)->id () < (*it2)->id())
                return true;
              if ((*it1)->id () > (*it2)->id())
                return false;
              it2++;
            }
            return false;
          }

          bool operator==(const NodeBin& rhs) const {
            return nodes_ == rhs.nodes ();
          }

          const Nodes_t& nodes () const
          {
            return nodes_;
          }

          std::ostream& print (std::ostream& os) const
          {
            Parent::print (os) << " (";
            /// Sort by connected component.
            typedef std::list <RoadmapNodes_t> NodesList_t;
            NodesList_t l;
            bool found;
            for (RoadmapNodes_t::const_iterator itn = roadmapNodes_.begin ();
                itn != roadmapNodes_.end (); itn++) {
              found = false;
              for (NodesList_t::iterator itc = l.begin ();
                  itc != l.end (); itc++) {
                if ((*itn)->connectedComponent () == itc->front ()->connectedComponent ()) {
                  itc->push_back (*itn);
                  found = true;
                  break;
                }
              }
              if (!found) {
                l.push_back (RoadmapNodes_t (1, *itn));
              }
            }
            for (NodesList_t::iterator itc = l.begin ();
                itc != l.end (); itc++)
              os << itc->front ()->connectedComponent () << " - " << itc->size () << ", ";
            return os << ").";
          }

        private:
          Nodes_t nodes_;

          typedef std::list <core::NodePtr_t> RoadmapNodes_t;
          RoadmapNodes_t roadmapNodes_;

          std::ostream& printValue (std::ostream& os) const
          {
            os << "NodeBin (";
            Nodes_t::const_iterator it1;
            for (it1 = nodes_.begin(); it1 != nodes_.end(); it1++)
              os << (*it1)->name () << " / ";
            return os << ")";
          }
      };

      class Histogram;
      typedef boost::shared_ptr <Histogram> HistogramPtr_t;

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
          LeafHistogram (const ConstraintSetPtr_t& constraint) :
            constraint_ (constraint) {}

          /// Insert an occurence of a value in the histogram
          void add (const core::NodePtr_t& n)
          {
            LeafBin b(constraint_->offsetFromConfig (*n->configuration ()));
            increment (b);
            b.push_back (n);
            if (numberOfObservations()%10 == 0) {
              hppDout (info, *this);
            }
          }

          std::ostream& print (std::ostream& os) const
          {
            os << "Histogram constains ConstraintSet: "
              << constraint_->name () << std::endl;
            return Parent::print (os);
          }

          const ConstraintSetPtr_t& constraint () const
          {
            return constraint_;
          }

          virtual HistogramPtr_t clone () const
          {
            return HistogramPtr_t (new LeafHistogram (constraint_));
          }

        private:
          /// The constraint that creates the foliation.
          ConstraintSetPtr_t constraint_;
      };

      class HPP_MANIPULATION_DLLLOCAL NodeHistogram : public ::hpp::statistics::Statistics < NodeBin >
                                                      , public Histogram
      {
        public:
          typedef ::hpp::statistics::Statistics < NodeBin > Parent;
          /// Constructor
          /// \param graph The constraint graph used to get the nodes from
          ///        a configuration.
          NodeHistogram (const graph::GraphPtr_t& graph) :
            graph_ (graph) {}

          /// Insert an occurence of a value in the histogram
          void add (const core::NodePtr_t& n)
          {
            NodeBin b(graph_->getNode (*n->configuration ()));
            increment (b);
            b.push_back (n);
            if (numberOfObservations()%10 == 0) {
              hppDout (info, *this);
            }
          }

          std::ostream& print (std::ostream& os) const
          {
            os << "Graph Node Histogram constains: " << std::endl;
            return Parent::print (os);
          }

          const graph::GraphPtr_t& constraintGraph () const
          {
            return graph_;
          }

          virtual HistogramPtr_t clone () const
          {
            return HistogramPtr_t (new NodeHistogram (graph_));
          }

        private:
          /// The constraint graph
          graph::GraphPtr_t graph_;
      };
    } // namespace graph
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_GRAPH_STATISTICS_HH
