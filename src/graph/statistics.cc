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

#include "hpp/manipulation/constraint-set.hh"

namespace hpp {
  namespace manipulation {
    namespace graph {
      LeafBin::LeafBin(const vector_t& v, value_type* thr):
        value_(v), nodes_(), thr_ (thr)
      {}

      void LeafBin::push_back(const RoadmapNodePtr_t& n)
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

      NodeBin::NodeBin(const StatePtr_t& n):
        state_(n), roadmapNodes_()
      {}

      void NodeBin::push_back(const RoadmapNodePtr_t& n)
      {
        roadmapNodes_.push_back(n);
      }

      bool NodeBin::operator<(const NodeBin& rhs) const
      {
        return state_->id () < rhs.state ()->id ();
      }

      bool NodeBin::operator==(const NodeBin& rhs) const
      {
        return state_ == rhs.state ();
      }

      const StatePtr_t& NodeBin::state () const
      {
        return state_;
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
        return os << "NodeBin (" << state_->name () << ")";
      }

      LeafHistogramPtr_t LeafHistogram::create (const Foliation f)
      {
        return LeafHistogramPtr_t (new LeafHistogram (f));
      }

      LeafHistogram::LeafHistogram (const Foliation f) :
        f_ (f), threshold_ (0)
      {
        ConfigProjectorPtr_t p = f_.parametrizer ()->configProjector();
        if (p) {
          if (p->rightHandSide ().size () > 0)
            threshold_ = p->errorThreshold () /
              sqrt((double)p->rightHandSide ().size ());
        }
      }

      void LeafHistogram::add (const RoadmapNodePtr_t& n)
      {
        if (!f_.contains (*n->configuration())) return;
	iterator it = insert (LeafBin (f_.parameter (*n->configuration()),
                              &threshold_));
        it->push_back (n);
        if (numberOfObservations()%10 == 0) {
          hppDout (info, *this);
        }
      }

      std::ostream& LeafHistogram::print (std::ostream& os) const
      {
        os << "Leaf Histogram of foliation " << f_.condition()->name() << std::endl;
        return Parent::print (os);
      }

      HistogramPtr_t LeafHistogram::clone () const
      {
        return HistogramPtr_t (new LeafHistogram (f_));
      }

      StateHistogram::StateHistogram (const graph::GraphPtr_t& graph) :
        graph_ (graph) {}

      void StateHistogram::add (const RoadmapNodePtr_t& n)
      {
        iterator it = insert (NodeBin (graph_->getState (n)));
        it->push_back (n);
        if (numberOfObservations()%10 == 0) {
          hppDout (info, *this);
        }
      }

      std::ostream& StateHistogram::print (std::ostream& os) const
      {
        os << "Graph State Histogram contains: " << std::endl;
        return Parent::print (os);
      }

      const graph::GraphPtr_t& StateHistogram::constraintGraph () const
      {
        return graph_;
      }

      HistogramPtr_t StateHistogram::clone () const
      {
        return HistogramPtr_t (new StateHistogram (graph_));
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

      statistics::DiscreteDistribution < RoadmapNodePtr_t > LeafHistogram::getDistribOutOfConnectedComponent (
          const core::ConnectedComponentPtr_t& cc) const
      {
        statistics::DiscreteDistribution < RoadmapNodePtr_t > distrib;
        for (const_iterator bin = begin(); bin != end (); ++bin) {
          unsigned int w = bin->numberOfObsOutOfConnectedComponent (cc);
          if (w == 0)
            continue;
          distrib.insert (bin->nodes ().front (), w);
        }
        return distrib;
      }

      statistics::DiscreteDistribution < RoadmapNodePtr_t > LeafHistogram::getDistrib () const
      {
        statistics::DiscreteDistribution < RoadmapNodePtr_t > distrib;
        for (const_iterator bin = begin(); bin != end (); ++bin) {
          unsigned int w = bin->freq ();
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

      bool Foliation::contains (ConfigurationIn_t q) const
      {
        return condition_->isSatisfied (q);
      }

      vector_t Foliation::parameter (ConfigurationIn_t q) const
      {
        if (!condition_->isSatisfied (q)) {
          hppDout (error, "Configuration not in the foliation");
        }
        return parametrizer_->configProjector()->rightHandSideFromConfig (q);
      }

      ConstraintSetPtr_t Foliation::condition () const
      {
        return condition_;
      }

      void Foliation::condition (const ConstraintSetPtr_t c)
      {
        condition_ = c;
      }

      ConstraintSetPtr_t Foliation::parametrizer () const
      {
        return parametrizer_;
      }

      void Foliation::parametrizer (const ConstraintSetPtr_t p)
      {
        parametrizer_ = p;
      }
    } // namespace graph
  } // namespace manipulation
} // namespace hpp
