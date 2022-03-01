// Copyright (c) 2018, Joseph Mirabel
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
          if (it->second->stateTo () == to)
            edges.push_back (it->second);
        }
        for (Edges_t::const_iterator it = from->hiddenNeighbors ().begin ();
            it != from->hiddenNeighbors ().end (); ++it) {
          if ((*it)->stateTo () == to)
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
          StatePtr_t src = c->edge()->stateFrom();
          StatePtr_t dst = c->edge()->stateTo  ();
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
