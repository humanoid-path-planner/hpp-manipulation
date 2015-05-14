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

#include "hpp/manipulation/graph/statistics.hh"

namespace hpp {
  namespace manipulation {
    namespace graph {
      LeafBin::LeafBin(const vector_t& v, value_type* thr):
        value_(v), nodes_(), thr_ (thr)
      {}

      void LeafBin::push_back(const core::NodePtr_t& n)
      {
        nodes_.push_back(n);
      }

      bool LeafBin::operator<(const LeafBin& rhs) const
      {
        const vector_t& v = rhs.value ();
        assert (value_.size() == v.size());
        for (int p = 0; p < value_.size(); p++) {
          if (std::abs (value_[p] - v[p]) >= *thr_)
            return value_[p] < v[p];
        }
        return false;
      }

      bool LeafBin::operator==(const LeafBin& rhs) const
      {
        const vector_t& v = rhs.value ();
        assert (value_.size() == v.size());
        for (int p = 0; p < value_.size(); p++) {
          if (std::abs (value_[p] - v[p]) >= *thr_)
            return false;
        }
        return true;
      }

      const vector_t& LeafBin::value () const
      {
        return value_;
      }

      std::ostream& LeafBin::print (std::ostream& os) const
      {
        Parent::print (os) << " (";
        /// Sort by connected component.
        typedef std::list <RoadmapNodes_t> NodesList_t;
        NodesList_t l;
        bool found;
        for (RoadmapNodes_t::const_iterator itn = nodes_.begin ();
            itn != nodes_.end (); ++itn) {
          found = false;
          for (NodesList_t::iterator itc = l.begin ();
              itc != l.end (); ++itc) {
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
            itc != l.end (); ++itc)
          os << itc->front ()->connectedComponent () << " - " << itc->size () << ", ";
        return os << ").";
      }

      std::ostream& LeafBin::printValue (std::ostream& os) const
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

      NodeBin::NodeBin(const NodePtr_t& n):
        node_(n), roadmapNodes_()
      {}

      void NodeBin::push_back(const core::NodePtr_t& n)
      {
        roadmapNodes_.push_back(n);
      }

      bool NodeBin::operator<(const NodeBin& rhs) const
      {
        return node_->id () < rhs.node ()->id ();
      }

      bool NodeBin::operator==(const NodeBin& rhs) const
      {
        return node_ == rhs.node ();
      }

      const NodePtr_t& NodeBin::node () const
      {
        return node_;
      }

      std::ostream& NodeBin::print (std::ostream& os) const
      {
        Parent::print (os) << " (";
        /// Sort by connected component.
        typedef std::list <RoadmapNodes_t> NodesList_t;
        NodesList_t l;
        bool found;
        for (RoadmapNodes_t::const_iterator itn = roadmapNodes_.begin ();
            itn != roadmapNodes_.end (); ++itn) {
          found = false;
          for (NodesList_t::iterator itc = l.begin ();
              itc != l.end (); ++itc) {
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
            itc != l.end (); ++itc)
          os << itc->front ()->connectedComponent () << " - " << itc->size () << ", ";
        return os << ").";
      }

      std::ostream& NodeBin::printValue (std::ostream& os) const
      {
        return os << "NodeBin (" << node_->name () << ")";
      }

      LeafHistogram::LeafHistogram (const ConstraintSetPtr_t& constraint) :
        constraint_ (constraint), threshold_ (0)
      {
        ConfigProjectorPtr_t p = constraint_->configProjector ();
        if (p) {
          threshold_ = p->errorThreshold () /
	    sqrt((double)p->rightHandSide ().size ());
        }
      }

      void LeafHistogram::add (const core::NodePtr_t& n)
      {
	iterator it;
	if (constraint_->configProjector ()) {
	  it = insert
	    (LeafBin (constraint_->configProjector ()->rightHandSideFromConfig
		      (*n->configuration ()),
                      &threshold_));
	} else {
	  it = insert (LeafBin (vector_t (), &threshold_));
	}
        it->push_back (n);
        if (numberOfObservations()%10 == 0) {
          hppDout (info, *this);
        }
      }

      std::ostream& LeafHistogram::print (std::ostream& os) const
      {
        os << "Histogram contains ConstraintSet: "
          << constraint_->name () << std::endl;
        return Parent::print (os);
      }

      const ConstraintSetPtr_t& LeafHistogram::constraint () const
      {
        return constraint_;
      }

      HistogramPtr_t LeafHistogram::clone () const
      {
        return HistogramPtr_t (new LeafHistogram (constraint_));
      }

      NodeHistogram::NodeHistogram (const graph::GraphPtr_t& graph) :
        graph_ (graph) {}

      void NodeHistogram::add (const core::NodePtr_t& n)
      {
        iterator it = insert (NodeBin (graph_->getNode (*n->configuration ())));
        it->push_back (n);
        if (numberOfObservations()%10 == 0) {
          hppDout (info, *this);
        }
      }

      std::ostream& NodeHistogram::print (std::ostream& os) const
      {
        os << "Graph Node Histogram contains: " << std::endl;
        return Parent::print (os);
      }

      const graph::GraphPtr_t& NodeHistogram::constraintGraph () const
      {
        return graph_;
      }

      HistogramPtr_t NodeHistogram::clone () const
      {
        return HistogramPtr_t (new NodeHistogram (graph_));
      }

      unsigned int LeafBin::numberOfObsOutOfConnectedComponent (const core::ConnectedComponentPtr_t& cc) const
      {
        unsigned int count = 0;
        for (RoadmapNodes_t::const_iterator it = nodes_.begin ();
            it != nodes_.end (); ++it)
          if ((*it)->connectedComponent () != cc)
            count++;
        return count;
      }

      statistics::DiscreteDistribution < core::NodePtr_t > LeafHistogram::getDistribOutOfConnectedComponent (
          const core::ConnectedComponentPtr_t& cc) const
      {
        statistics::DiscreteDistribution < core::NodePtr_t > distrib;
        for (const_iterator bin = begin(); bin != end (); ++bin) {
          unsigned int w = bin->numberOfObsOutOfConnectedComponent (cc);
          if (w == 0)
            continue;
          distrib.insert (bin->nodes ().front (), w);
        }
        return distrib;
      }

      const LeafBin::RoadmapNodes_t& LeafBin::nodes () const
      {
        return nodes_;
      }
    } // namespace graph
  } // namespace manipulation
} // namespace hpp
