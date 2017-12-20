// Copyright (c) 2017, Joseph Mirabel
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

#include <hpp/manipulation/steering-method/cross-state-optimization.hh>

#include <map>
#include <queue>
#include <vector>

#include <hpp/manipulation/graph/edge.hh>
#include <hpp/manipulation/graph/state.hh>

namespace hpp {
  namespace manipulation {
    namespace steeringMethod {
      using namespace graph;

      CrossStateOptimizationPtr_t CrossStateOptimization::create (
          const Problem& problem)
      {
        CrossStateOptimizationPtr_t shPtr (new CrossStateOptimization (problem));
        shPtr->init(shPtr);
        return shPtr;
      }

      CrossStateOptimizationPtr_t CrossStateOptimization::createFromCore (
          const core::Problem& problem)
      {
        HPP_STATIC_CAST_REF_CHECK (const Problem, problem);
        const Problem& p = static_cast <const Problem&> (problem);
        return create (p);
      }

      core::SteeringMethodPtr_t CrossStateOptimization::copy () const
      {
        CrossStateOptimization* ptr = new CrossStateOptimization (*this);
        CrossStateOptimizationPtr_t shPtr (ptr);
        ptr->init (shPtr);
        return shPtr;
      }

      struct CrossStateOptimization::Data {
        typedef graph::StatePtr_t StatePtr_t;
        StatePtr_t s1, s2;

        // Datas for findNextTransitions
        struct state_with_depth {
          StatePtr_t s;
          EdgePtr_t e;
          std::size_t l; // depth to root
          std::size_t i; // index in parent state_with_depths_t
          inline state_with_depth () : s(), e(), l(0), i (0) {}
          inline state_with_depth (EdgePtr_t _e, std::size_t _l, std::size_t _i)
            : s(_e->from()), e(_e), l(_l), i (_i) {}
        };
        typedef std::vector<state_with_depth> state_with_depths_t;
        typedef std::map<StatePtr_t,state_with_depths_t> StateMap_t;
        typedef std::pair<StateMap_t::iterator, std::size_t> state_with_depth_ptr_t;
        typedef std::queue<state_with_depth_ptr_t> Queue_t;
        typedef std::set<EdgePtr_t> VisitedEdge_t;
        std::size_t maxDepth;
        StateMap_t parent1; // TODO, parent2;
        Queue_t queue1;
        VisitedEdge_t visitedEdge_;

        const state_with_depth& getParent(const state_with_depth_ptr_t& _p) const
        {
          const state_with_depths_t& parents = _p.first->second;
          return parents[_p.second];
        }

        state_with_depth_ptr_t addInitState()
        {
          StateMap_t::iterator next =
            parent1.insert(StateMap_t::value_type(s1, state_with_depths_t(1))).first;
          return state_with_depth_ptr_t (next, 0);
        }

        state_with_depth_ptr_t addParent(
            const state_with_depth_ptr_t& _p,
            const EdgePtr_t& transition)
        {
          const state_with_depths_t& parents = _p.first->second;
          const state_with_depth& from = parents[_p.second];

          // Insert state to if necessary
          StateMap_t::iterator next = parent1.insert (
              StateMap_t::value_type(
                transition->to(),
                Data::state_with_depths_t ()
                )).first;

          next->second.push_back (
              state_with_depth(transition, from.l + 1, _p.second));

          return state_with_depth_ptr_t (next, next->second.size()-1);
        }
      };

      core::PathPtr_t CrossStateOptimization::impl_compute (
          ConfigurationIn_t q1, ConfigurationIn_t q2) const
      {
        const Graph& graph = *problem_.constraintGraph ();
        Data d;
        d.s1 = graph.getState (q1);
        d.s2 = graph.getState (q2);
        d.maxDepth = 2;

        // Find 
        d.queue1.push (d.addInitState());
        std::size_t idxSol = (d.s1 == d.s2 ? 1 : 0);
        bool maxDepthReached = findTransitions (d);

        while (!maxDepthReached) {
          Edges_t transitions = getTransitionList (d, idxSol);
          while (! transitions.empty()) {
            std::cout << "Solution " << idxSol << ": ";
            for (std::size_t j = 0; j < transitions.size(); ++j)
              std::cout << transitions[j]->name() << ", ";
            std::cout << std::endl;
            ++idxSol;
            transitions = getTransitionList(d, idxSol);
          }
          maxDepthReached = findTransitions (d);
        }

        return core::PathPtr_t ();
      }

      bool CrossStateOptimization::findTransitions (Data& d) const
      {
        while (! d.queue1.empty())
        {
          Data::state_with_depth_ptr_t _state = d.queue1.front();

          const Data::state_with_depth& parent = d.getParent(_state);
          if (parent.l >= d.maxDepth) return true;
          d.queue1.pop();

          bool done = false;

          const Neighbors_t& neighbors = _state.first->first->neighbors();
          for (Neighbors_t::const_iterator _n = neighbors.begin();
              _n != neighbors.end(); ++_n) {
            EdgePtr_t transition = _n->second;

            // If transition has already been visited, continue
            // if (d.visitedEdge_.count (transition) == 1) continue;

            // Insert parent
            d.queue1.push (
                d.addParent (_state, transition)
                );

            done = done || (transition->to() == d.s2);
          }
          if (done) break;
        }
        return false;
      }

      Edges_t CrossStateOptimization::getTransitionList (
          Data& d, const std::size_t& i) const
      {
        assert (d.parent1.find (d.s2) != d.parent1.end());
        const Data::state_with_depths_t& roots = d.parent1[d.s2];
        Edges_t transitions;
        if (i >= roots.size()) return transitions;

        const Data::state_with_depth* current = &roots[i];
        transitions.resize (current->l);
        while (current->e) {
          assert (current->l > 0);
          transitions[current->l-1] = current->e;
          current = &d.parent1[current->s][current->i];
        }
        return transitions;
      }
    } // namespace steeringMethod
  } // namespace manipulation
} // namespace hpp
