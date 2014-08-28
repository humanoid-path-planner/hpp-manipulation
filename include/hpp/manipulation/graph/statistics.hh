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
          LeafBin(const vector_t& v): value_(v), configs_() {}

          void push_back(const Configuration_t& cfg) {
            configs_.push_back(cfg);
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

        private:
          vector_t value_;

          std::list <Configuration_t> configs_;

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
          NodeBin(const ConstraintSetPtr_t& c): constraint_(c), configs_() {}

          void push_back(const Configuration_t& cfg) {
            configs_.push_back(cfg);
          }

          bool operator<(const NodeBin& rhs) const {
            return constraint_.get() < rhs.constraint ().get();
          }

          bool operator==(const NodeBin& rhs) const {
            return constraint_ == rhs.constraint ();
          }

          const ConstraintSetPtr_t& constraint () const {
            return constraint_;
          }

        private:
          ConstraintSetPtr_t constraint_;

          std::list <Configuration_t> configs_;

          std::ostream& printValue (std::ostream& os) const
          {
            return os << "NodeBin (" << constraint_->name () << ")";
          }
      };

      class Histogram;
      typedef boost::shared_ptr <Histogram> HistogramPtr_t;

      class HPP_MANIPULATION_DLLLOCAL Histogram
      {
        public:
          virtual void add (const Configuration_t& config) = 0;

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
          void add (const Configuration_t& config)
          {
            LeafBin b(constraint_->offsetFromConfig (config));
            increment (b);
            b.push_back (config);
          }

          std::ostream& print (std::ostream& os) const
          {
            os << "Histogram constains ConstraintSet: ";
            os << *constraint_ << std::endl;
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
    } // namespace graph
  } // namespace manipulation

  std::ostream& operator<< (std::ostream& os, const manipulation::graph::LeafHistogram& h);
} // namespace hpp

#endif // HPP_MANIPULATION_GRAPH_STATISTICS_HH
