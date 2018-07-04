// Copyright (c) 2018, Joseph Mirabel
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

#include <hpp/manipulation/path-optimization/enforce-transition-semantic.hh>

#include <hpp/pinocchio/util.hh>
#include <hpp/core/path-vector.hh>

#include <hpp/manipulation/constraint-set.hh>
#include <hpp/manipulation/graph/edge.hh>
#include <hpp/manipulation/graph/graph.hh>
#include <hpp/manipulation/graph/state.hh>

namespace hpp {
  namespace manipulation {
    namespace pathOptimization {
      using hpp::core::Path;
      using hpp::core::PathPtr_t;
      using hpp::core::PathVector;
      using hpp::core::PathVectorPtr_t;
      using graph::Edges_t;
      using graph::StatePtr_t;


      Edges_t getAllEdges (const StatePtr_t& from, const StatePtr_t& to)
      {
        Edges_t edges;
        for (graph::Neighbors_t::const_iterator it = from->neighbors ().begin ();
            it != from->neighbors ().end (); ++it) {
          if (it->second->to () == to)
            edges.push_back (it->second);
        }
        for (Edges_t::const_iterator it = from->hiddenNeighbors ().begin ();
            it != from->hiddenNeighbors ().end (); ++it) {
          if ((*it)->to () == to)
            edges.push_back (*it);
        }
        return edges;
      }

      PathVectorPtr_t EnforceTransitionSemantic::optimize (const PathVectorPtr_t& path)
      {
        PathVectorPtr_t input = PathVector::create (
              path->outputSize(), path->outputDerivativeSize()); 
        PathVectorPtr_t output = PathVector::create (
              path->outputSize(), path->outputDerivativeSize()); 
        path->flatten (input);

        ConstraintSetPtr_t c;
        for (std::size_t i = 0; i < input->numberPaths(); ++i) {
          PathPtr_t current = input->pathAtRank (i);
          output->appendPath(current);
          c = HPP_DYNAMIC_PTR_CAST (ConstraintSet, current->constraints ());
          if (!c) {
            hppDout(info, "No manipulation::ConstraintSet");
            break;
          }
          Configuration_t q0 = current->initial();
          Configuration_t q1 = current->end    ();
          StatePtr_t src = c->edge()->from();
          StatePtr_t dst = c->edge()->to  ();
          if (src == dst) continue;

          bool q0_in_src = src->contains(q0);
          bool q1_in_src = src->contains(q1);
          bool q0_in_dst = dst->contains(q0);
          bool q1_in_dst = dst->contains(q1);

          if (q0_in_src && q1_in_dst) // Nominal case
            continue;
          hppDout (warning, "Transition " << i << ". "
              "\nsrc=" << src->name() <<
              "\ndst=" << dst->name() <<
              "\nq0_in_src=" << q0_in_src <<
              "\nq1_in_src=" << q1_in_src <<
              "\nq0_in_dst=" << q0_in_dst <<
              "\nq1_in_dst=" << q1_in_dst <<
              setpyformat <<
              "\nq0=" << one_line(q0) <<
              "\nq1=" << one_line(q1) <<
              unsetpyformat <<
              "\nTrying with state.");

          StatePtr_t from, to;
          if (q0_in_dst && q1_in_src) { // Reversed from nominal case
            from = dst;
            to = src;
          } else if (q0_in_dst && q1_in_dst) {
            from = dst;
            to = dst;
          } else if (q0_in_src && q1_in_src) {
            from = src;
            to = src;
          } else if (q0_in_src) { // Keep same transition
            continue;
          } else if (q0_in_dst) { // Reverse current transition
            from = dst;
            to = src;
          } else if (q1_in_src) { // Keep same transition
            continue;
          } else if (q1_in_dst) { // Reverse current transition
            from = dst;
            to = src;
          }
          if (from && to) {
            // Check that a path from dst to to exists.
            Edges_t transitions = getAllEdges(from, to);
            if (transitions.size() >= 1)
            {
              if (transitions.size() > 1)
              {
                hppDout (info, "More than one transition...");
              }
              c->edge(transitions[0]);
              continue;
            }
          }
          hppDout (warning, "Enable to find a suitable transition for " << i << ". "
              "\nsrc=" << src->name() <<
              "\ndst=" << dst->name() <<
              "\nq0_in_src=" << q0_in_src <<
              "\nq1_in_src=" << q1_in_src <<
              "\nq0_in_dst=" << q0_in_dst <<
              "\nq1_in_dst=" << q1_in_dst <<
              setpyformat <<
              "\nq0=" << one_line(q0) <<
              "\nq1=" << one_line(q1) <<
              unsetpyformat <<
              "\nTrying with state.");

          StatePtr_t state = c->edge()->state();
          // Check that a path from dst to to exists.
          Edges_t transitions = getAllEdges(state, state);
          if (transitions.size() >= 1)
          {
            if (transitions.size() > 1)
            {
              hppDout (info, "More than one transition...");
            }
            c->edge(transitions[0]);
            continue;
          } else {
            hppDout (error, "Enable to find a suitable transition for " << *current);
          }
        }
        return output;
      }

    } // namespace pathOptimization
  } // namespace manipulation
} // namespace hpp
