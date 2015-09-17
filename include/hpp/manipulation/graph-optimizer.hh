// Copyright (c) 2015, LAAS-CNRS
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


#ifndef HPP_MANIPULATION_GRAPHOPTIMIZER_HH
# define HPP_MANIPULATION_GRAPHOPTIMIZER_HH

# include <hpp/core/path.hh>
# include <hpp/core/path-vector.hh>
# include <hpp/core/path-optimizer.hh>
# include <hpp/core/problem.hh>

# include <hpp/manipulation/fwd.hh>
# include <hpp/manipulation/graph/fwd.hh>
# include <hpp/manipulation/config.hh>
# include <hpp/manipulation/constraint-set.hh>

namespace hpp {
  namespace manipulation {
    using hpp::core::PathOptimizer;
    using hpp::core::Path;
    using hpp::core::PathPtr_t;
    using hpp::core::PathVector;
    using hpp::core::PathVectorPtr_t;

    /// \addtogroup path_optimization
    /// \{

    /// Path optimizer for paths created with the constraint graph
    ///
    /// This class encapsulates another path optimizer class. This optimizer
    /// calls the inner optimizer on every subpaths with the same set of
    /// constraints.
    template <typename InnerPathOptimizer_t>
    class HPP_MANIPULATION_DLLAPI GraphOptimizer : public PathOptimizer
    {
      public:
        typedef boost::shared_ptr <InnerPathOptimizer_t> InnerPtr_t;
        typedef boost::shared_ptr <GraphOptimizer> Ptr_t;

        static Ptr_t create (const core::Problem& problem);

        // static Ptr_t create (const Problem& problem);

        virtual PathVectorPtr_t optimize (const PathVectorPtr_t& path);

        /// Get the encapsulated optimizer
        const InnerPtr_t& innerOptimizer ()
        {
          return pathOptimizer_;
        }

      protected:
        /// Constructor
        GraphOptimizer (const Problem& problem) :
          PathOptimizer (problem),
          pathOptimizer_ (InnerPathOptimizer_t::create (problem))
        {}

      private:
        /// The encapsulated PathOptimizer
        InnerPtr_t pathOptimizer_;

        /// Append all paths of in to out
        /// \param in a path vector, possibly containing other path vector
        /// \param out a flat PathVector (do not contain PathVector)
        static void unpack (PathVectorPtr_t in, PathVectorPtr_t out);
    };
    /// \}

    /// Member function definition
    template <typename InnerPathOptimizer_t>
      typename GraphOptimizer<InnerPathOptimizer_t>::Ptr_t
      GraphOptimizer<InnerPathOptimizer_t>::create (const core::Problem& problem)
    {
      return GraphOptimizer<InnerPathOptimizer_t>::Ptr_t (
          new GraphOptimizer<InnerPathOptimizer_t> (dynamic_cast <const Problem&> (problem))
          );
      // return GraphOptimizer<InnerPathOptimizer_t>::create
        // (dynamic_cast <const Problem&> (problem));
    }

    // template <typename InnerPathOptimizer_t>
      // typename GraphOptimizer<InnerPathOptimizer_t>::Ptr_t
      // GraphOptimizer<InnerPathOptimizer_t>::create (const Problem& problem)
    // {
      // return GraphOptimizer<InnerPathOptimizer_t>::Ptr_t (
          // new GraphOptimizer<InnerPathOptimizer_t> (problem));
    // }

    template <typename InnerPathOptimizer_t> PathVectorPtr_t
      GraphOptimizer<InnerPathOptimizer_t>::optimize (const PathVectorPtr_t& path)
    {
      PathVectorPtr_t opted = PathVector::create
                             (path->outputSize(), path->outputDerivativeSize()),
                      expanded = PathVector::create
                             (path->outputSize(), path->outputDerivativeSize()),
                      toConcat;
      unpack (path, expanded);
      ConstraintSetPtr_t c;
      for (std::size_t i_s = 0; i_s < expanded->numberPaths ();) {
        PathVectorPtr_t toOpt = PathVector::create (
            path->outputSize(), path->outputDerivativeSize()); 
        PathPtr_t current = expanded->pathAtRank (i_s);
        toOpt->appendPath (current);
        graph::EdgePtr_t edge;
        c = HPP_DYNAMIC_PTR_CAST (ConstraintSet, current->constraints ());
        if (c) edge = c->edge ();
        std::size_t i_e = i_s + 1;
        for (; i_e < expanded->numberPaths (); ++i_e) {
          current = expanded->pathAtRank (i_e);
          c = HPP_DYNAMIC_PTR_CAST (ConstraintSet, current->constraints ());
          if (!c && edge) break;
          if (c && edge != c->edge ()) break;
          toOpt->appendPath (current);
        }
        toConcat = pathOptimizer_->optimize (toOpt);
        i_s = i_e;
        opted->concatenate (*toConcat);
      }
      return opted;
    }

    template <typename InnerPathOptimizer_t>
      void GraphOptimizer<InnerPathOptimizer_t>::unpack
      (PathVectorPtr_t in, PathVectorPtr_t out)
    {
      for (size_t i = 0; i != in->numberPaths (); i++) {
        PathPtr_t current = in->pathAtRank (i);
        PathVectorPtr_t pv = HPP_DYNAMIC_PTR_CAST (PathVector, current);
        if (pv) {
          unpack (pv, out);
        } else {
          out->appendPath (current);
        }
      }
    }
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_GRAPHOPTIMIZER_HH
