// Copyright (c) 2017, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr),
//          Florent Lamiraux (florent.lamiraux@laas.fr)
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

#include <hpp/manipulation/steering-method/cross-state-optimization.hh>

#include <map>
#include <queue>
#include <vector>

#include <hpp/util/exception-factory.hh>

#include <pinocchio/multibody/model.hpp>

#include <hpp/pinocchio/configuration.hh>
#include <hpp/pinocchio/joint-collection.hh>

#include <hpp/constraints/affine-function.hh>
#include <hpp/constraints/locked-joint.hh>
#include <hpp/constraints/solver/by-substitution.hh>
#include <hpp/constraints/explicit.hh>

#include <hpp/core/path-vector.hh>
#include <hpp/core/configuration-shooter.hh>

#include <hpp/manipulation/graph/edge.hh>
#include <hpp/manipulation/graph/state.hh>

namespace hpp {
  namespace manipulation {
    namespace steeringMethod {
      using Eigen::RowBlockIndices;
      using Eigen::ColBlockIndices;

      using graph::StatePtr_t;
      using graph::States_t;
      using graph::EdgePtr_t;
      using graph::Edges_t;
      using graph::Neighbors_t;
      using graph::NumericalConstraints_t;
      using graph::LockedJoints_t;
      using graph::segments_t;

      CrossStateOptimizationPtr_t CrossStateOptimization::create (
          const ProblemConstPtr_t& problem)
      {
        CrossStateOptimizationPtr_t shPtr(new CrossStateOptimization (problem));
        shPtr->init(shPtr);
        return shPtr;
      }

      CrossStateOptimizationPtr_t CrossStateOptimization::create (
          const core::ProblemConstPtr_t& problem)
      {
        assert(HPP_DYNAMIC_PTR_CAST(const Problem, problem));
        ProblemConstPtr_t p(HPP_STATIC_PTR_CAST(const Problem, problem));
        return create (p);
      }

      core::SteeringMethodPtr_t CrossStateOptimization::copy () const
      {
        CrossStateOptimization* ptr = new CrossStateOptimization (*this);
        CrossStateOptimizationPtr_t shPtr (ptr);
        ptr->init (shPtr);
        return shPtr;
      }

      struct CrossStateOptimization::GraphSearchData
      {
        StatePtr_t s1, s2;

        // Datas for findNextTransitions
        struct state_with_depth {
          StatePtr_t s;
          EdgePtr_t e;
          std::size_t l; // depth to root
          std::size_t i; // index in parent state_with_depths_t
          inline state_with_depth () : s(), e(), l(0), i (0) {}
          inline state_with_depth (EdgePtr_t _e, std::size_t _l, std::size_t _i)
            : s(_e->stateFrom()), e(_e), l(_l), i (_i) {}
        };
        typedef std::vector<state_with_depth> state_with_depths_t;
        typedef std::map<StatePtr_t,state_with_depths_t> StateMap_t;
        /// std::size_t is the index in state_with_depths_t at StateMap_t::iterator
        struct state_with_depth_ptr_t {
          StateMap_t::iterator state;
          std::size_t parentIdx;
          state_with_depth_ptr_t (const StateMap_t::iterator& it, std::size_t idx) : state (it), parentIdx (idx) {}
        };
        typedef std::queue<state_with_depth_ptr_t> Queue_t;
        typedef std::set<EdgePtr_t> VisitedEdge_t;
        std::size_t maxDepth;
        StateMap_t parent1; // TODO, parent2;
        Queue_t queue1;
        VisitedEdge_t visitedEdge_;

        const state_with_depth& getParent(const state_with_depth_ptr_t& _p) const
        {
          const state_with_depths_t& parents = _p.state->second;
          return parents[_p.parentIdx];
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
          const state_with_depths_t& parents = _p.state->second;
          const state_with_depth& from = parents[_p.parentIdx];

          // Insert state to if necessary
          StateMap_t::iterator next = parent1.insert (
              StateMap_t::value_type(
                transition->stateTo(), state_with_depths_t ()
                )).first;

          next->second.push_back (
              state_with_depth(transition, from.l + 1, _p.parentIdx));

          return state_with_depth_ptr_t (next, next->second.size()-1);
        }
      };

      void CrossStateOptimization::gatherGraphConstraints ()
      {
        typedef graph::Edge Edge;
        typedef graph::EdgePtr_t EdgePtr_t;
        typedef graph::GraphPtr_t GraphPtr_t;
        typedef constraints::solver::BySubstitution Solver_t;

        GraphPtr_t cg (problem_->constraintGraph ());
        const ConstraintsAndComplements_t& cac
          (cg->constraintsAndComplements ());
        for (std::size_t i = 0; i < cg->nbComponents (); ++i) {
          EdgePtr_t edge (HPP_DYNAMIC_PTR_CAST (Edge, cg->get (i).lock ()));
          if (edge) {
            const Solver_t& solver (edge->pathConstraint ()->
                                    configProjector ()->solver ());
            const NumericalConstraints_t& constraints
              (solver.numericalConstraints ());
            for (NumericalConstraints_t::const_iterator it
                   (constraints.begin ()); it != constraints.end (); ++it) {
              if ((*it)->parameterSize () > 0) {
                const std::string& name ((*it)->function ().name  ());
                if (index_.find (name) == index_.end ()) {
                  // constraint is not in map, add it
                  index_ [name] = constraints_.size ();
                  // Check whether constraint is equivalent to a previous one
                  for (NumericalConstraints_t::const_iterator it1
                         (constraints_.begin ()); it1 != constraints_.end ();
                       ++it1) {
                    for (ConstraintsAndComplements_t::const_iterator it2
                           (cac.begin ()); it2 != cac.end (); ++it2) {
                      if (((**it1 == *(it2->complement)) &&
                           (**it == *(it2->both))) ||
                          ((**it1 == *(it2->both)) &&
                           (**it == *(it2->complement)))) {
                        assert (sameRightHandSide_.count (*it1) == 0);
                        assert (sameRightHandSide_.count (*it) == 0);
                        sameRightHandSide_ [*it1] = *it;
                        sameRightHandSide_ [*it] = *it1;
                      }
                    }
                  }
                  constraints_.push_back (*it);
                  hppDout (info, "Adding constraint \"" << name << "\"");
                  hppDout (info, "Edge \"" << edge->name () << "\"");
                  hppDout (info, "parameter size: " << (*it)->parameterSize ());

                }
              }
            }
          }
        }
      }

      static bool containsLevelSet(const graph::EdgePtr_t& e) {
        // First case, in case given edge e is already a sub edge inside a WaypointEdge
        graph::LevelSetEdgePtr_t lse =
          HPP_DYNAMIC_PTR_CAST(graph::LevelSetEdge, e);
        if (lse)
          return true;
        // Second case, given edge e links two non-waypoint states
        graph::WaypointEdgePtr_t we =
          HPP_DYNAMIC_PTR_CAST(graph::WaypointEdge, e);
        if (!we)
          return false;
        for (std::size_t i = 0; i <= we->nbWaypoints(); i++) {
          graph::LevelSetEdgePtr_t lse =
            HPP_DYNAMIC_PTR_CAST(graph::LevelSetEdge, we->waypoint(i));
          if (lse)
            return true;
        }
        return false;
      }

      static bool containsLevelSet(const graph::Edges_t& transitions) {
        for (std::size_t i = 0; i < transitions.size(); i++)
          if (HPP_DYNAMIC_PTR_CAST(graph::LevelSetEdge, transitions[i]))
            return true;
        return false;
      }

#ifdef LSE_GET_TRANSITION_LISTS
      /// Given an edge path "transitions", return a vector of 2^i edge paths
      /// where i is the number of edges in transitions which have a LSE alter ego
      static std::vector<graph::Edges_t> transitionsWithLSE(
          const graph::GraphPtr_t& graph, const graph::Edges_t& transitions) {
        std::vector<graph::Edges_t> altEdges(transitions.size());
        std::vector<std::size_t> indexWithAlt;

        for (std::size_t i = 0; i < transitions.size(); i++)
        {
          graph::StatePtr_t from = transitions[i]->stateFrom ();
          graph::StatePtr_t to = transitions[i]->stateTo ();
          altEdges[i] = graph->getEdges (from, to);
          if (altEdges[i].size() == 2)
            indexWithAlt.push_back (i);
        }
        std::size_t nbEdgesWithAlt = indexWithAlt.size();
        std::size_t nbAlternatives = ((std::size_t) 1) << nbEdgesWithAlt;

        std::vector<graph::Edges_t> alternativePaths(nbAlternatives);
        if (nbAlternatives == 1) {
          alternativePaths[0] = transitions;
          return alternativePaths;
        }

        for (std::size_t i = 0; i < nbAlternatives; i++)
        {
          std::size_t alti = 0;
          EdgePtr_t edge;
          graph::WaypointEdgePtr_t we;
          for (std::size_t j = 0; j < transitions.size(); j++)
          {
            if (j == indexWithAlt[alti])
            {
              edge = altEdges[j][(i >> alti) & 1];
              if (alti+1 != nbEdgesWithAlt)
                alti++;
            } else {
              edge = transitions[j];
            }
            we = HPP_DYNAMIC_PTR_CAST(graph::WaypointEdge, edge);
            if (we) {
              for (std::size_t k = 0; k <= we->nbWaypoints(); k++)
                alternativePaths[i].push_back(we->waypoint(k));
            } else {
              alternativePaths[i].push_back(edge);
            }
          }
        }
        return alternativePaths;
      }

      std::vector<Edges_t> CrossStateOptimization::getTransitionLists (
          const GraphSearchData& d, const std::size_t& i) const
      {
        assert (d.parent1.find (d.s2) != d.parent1.end());
        const GraphSearchData::state_with_depths_t& roots = d.parent1.at(d.s2);
        if (i >= roots.size()) return std::vector<Edges_t>();
        const GraphSearchData::state_with_depth* current = &roots[i];

        Edges_t transitions;
        transitions.reserve (current->l);
        while (current->e) {
          assert (current->l > 0);
          transitions.push_back(current->e);
          current = &d.parent1.at(current->s)[current->i];
        }
        std::reverse (transitions.begin(), transitions.end());
        return transitionsWithLSE(problem_->constraintGraph(), transitions);
      }
#endif
      bool CrossStateOptimization::findTransitions (GraphSearchData& d) const
      {
        while (! d.queue1.empty())
        {
          GraphSearchData::state_with_depth_ptr_t _state = d.queue1.front();

          const GraphSearchData::state_with_depth& parent = d.getParent(_state);
          if (parent.l >= d.maxDepth) return true;
          d.queue1.pop();

          bool done = false;

          const Neighbors_t& neighbors = _state.state->first->neighbors();
          for (Neighbors_t::const_iterator _n = neighbors.begin();
              _n != neighbors.end(); ++_n) {
            EdgePtr_t transition = _n->second;

#ifdef LSE_GET_TRANSITION_LISTS
            // Don't even consider level set edges
            if (containsLevelSet(transition)) continue;
#endif
            // Avoid identical consecutive transition
            if (transition == parent.e) continue;

            // If transition has already been visited, continue
            // if (d.visitedEdge_.count (transition) == 1) continue;

            // TODO
            // If (transition->to() == d.s2) check if this list is feasible.
            // - If a constraint with non-constant right hand side is present
            //   in all transitions, then the rhs from d.q1 and d.q2 should be
            //   equal

            // Insert parent
            d.queue1.push (
                d.addParent (_state, transition)
                );

            done = done || (transition->stateTo() == d.s2);
          }
          if (done) break;
        }
        return false;
      }

      Edges_t CrossStateOptimization::getTransitionList (
          const GraphSearchData& d, const std::size_t& i) const
      {
        assert (d.parent1.find (d.s2) != d.parent1.end());
        const GraphSearchData::state_with_depths_t& roots = d.parent1.at(d.s2);
        Edges_t transitions;
        if (i >= roots.size()) return transitions;

        const GraphSearchData::state_with_depth* current = &roots[i];
        transitions.reserve (current->l);
        graph::WaypointEdgePtr_t we;
        while (current->e) {
          assert (current->l > 0);
          we = HPP_DYNAMIC_PTR_CAST(graph::WaypointEdge, current->e);
          if (we) {
            for (int i = (int)we->nbWaypoints(); i >= 0; --i)
              transitions.push_back(we->waypoint(i));
          } else {
            transitions.push_back(current->e);
          }
          current = &d.parent1.at(current->s)[current->i];
        }
        std::reverse (transitions.begin(), transitions.end());
        return transitions;
      }

      namespace internal {
        bool saturate (const core::DevicePtr_t& robot, vectorIn_t q,
                       vectorOut_t qSat, pinocchio::ArrayXb& saturatedDof)
        {
          qSat = q;
          return hpp::pinocchio::saturate (robot, qSat, saturatedDof);
        }
      } // namespace internal

      struct CrossStateOptimization::OptimizationData
      {
        typedef constraints::solver::HierarchicalIterative::Saturation_t
        Saturation_t;
        enum RightHandSideStatus_t {
          // Constraint is not in solver for this waypoint
          ABSENT,
          // right hand side of constraint for this waypoint is equal to
          // right hand side for previous waypoint
          EQUAL_TO_PREVIOUS,
          // right hand side of constraint for this waypoint is equal to
          // right hand side for initial configuration
          EQUAL_TO_INIT,
          // right hand side of constraint for this waypoint is equal to
          // right hand side for goal configuration
          EQUAL_TO_GOAL
        }; // enum RightHandSideStatus_t
        const std::size_t N, nq, nv;
        std::vector <Solver_t> solvers;
        // Waypoints lying in each intermediate state
        matrix_t waypoint;
        // Initial guess of each solver stored as matrix columns
        matrix_t qInit;
        Configuration_t q1, q2;
        core::DevicePtr_t robot;
        // Matrix specifying for each constraint and each waypoint how
        // the right hand side is initialized in the solver.
        Eigen::Matrix < LiegroupElement, Eigen::Dynamic, Eigen::Dynamic > M_rhs;
        Eigen::Matrix < RightHandSideStatus_t, Eigen::Dynamic, Eigen::Dynamic >
        M_status;
        // Number of trials to generate each waypoint configuration
        OptimizationData (const core::ProblemConstPtr_t _problem,
                          const Configuration_t& _q1,
                          const Configuration_t& _q2,
                          const Edges_t& transitions
                          ) :
          N (transitions.size () - 1), nq (_problem->robot()->configSize()),
          nv (_problem->robot()->numberDof()),
          solvers (N, _problem->robot()->configSpace ()),
          waypoint (nq, N), qInit (nq, N), q1 (_q1), q2 (_q2),
          robot (_problem->robot()),
          M_rhs (), M_status ()
        {
          for (auto solver: solvers){
            // Set maximal number of iterations for each solver
            solver.maxIterations(_problem->getParameter
                            ("CrossStateOptimization/maxIteration").intValue());
            // Set error threshold for each solver
            solver.errorThreshold(_problem->getParameter
                        ("CrossStateOptimization/errorThreshold").floatValue());
          }
          assert (transitions.size () > 0);
        }
      };

      bool CrossStateOptimization::checkConstantRightHandSide
      (OptimizationData& d, size_type index) const
      {
        const ImplicitPtr_t c (constraints_ [index]);
        LiegroupElement rhsInit(c->function().outputSpace());
        c->rightHandSideFromConfig (d.q1, rhsInit);
        LiegroupElement rhsGoal(c->function().outputSpace());
        c->rightHandSideFromConfig (d.q2, rhsGoal);
        // Check that right hand sides are close to each other
        value_type eps (problem_->constraintGraph ()->errorThreshold ());
        value_type eps2 (eps * eps);
        if ((rhsGoal - rhsInit).squaredNorm () > eps2) {
          return false;
        }
        // Matrix of solver right hand sides
        for (size_type j=0; j<d.M_rhs.cols (); ++j) {
          d.M_rhs (index, j) = rhsInit;
        }
        return true;
      }

      void displayRhsMatrix (const Eigen::Matrix < vector_t, Eigen::Dynamic,
                             Eigen::Dynamic >& m,
                             const NumericalConstraints_t& constraints)
      {
        std::ostringstream oss; oss.precision (5);
        oss << "\\documentclass[12pt,landscape]{article}" << std::endl;
        oss << "\\usepackage[a3paper]{geometry}" << std::endl;
        oss << "\\begin {document}" << std::endl;
        oss << "\\begin {tabular}{";
        for (size_type j=0; j<m.cols () + 1; ++j)
          oss << "c";
        oss << "}" << std::endl;
        for (size_type i=0; i<m.rows (); ++i) {
          oss << constraints [i]->function ().name () << " & ";
          for (size_type j=0; j<m.cols (); ++j) {
            oss << "$\\left(\\begin{array}{c}" << std::endl;
            for (size_type k=0; k<m (i,j).size (); ++k) {
              oss << m (i,j) [k] << "\\\\" << std::endl;
            }
            oss << "\\end{array}\\right)$" << std::endl;
            if (j < m.cols () - 1) {
              oss << " & " << std::endl;
            }
          }
          oss << "\\\\" << std::endl;
        }
        oss << "\\end{tabular}" << std::endl;
        oss << "\\end {document}" << std::endl;
        hppDout (info, oss.str ());
      }

      void displayStatusMatrix (const Eigen::Matrix < CrossStateOptimization::OptimizationData::RightHandSideStatus_t, Eigen::Dynamic, Eigen::Dynamic >& m,
                                const NumericalConstraints_t& constraints,
                                const graph::Edges_t& transitions)
      {
        std::ostringstream oss; oss.precision (5);
        oss << "\\documentclass[12pt,landscape]{article}" << std::endl;
        oss << "\\usepackage[a3paper]{geometry}" << std::endl;
        oss << "\\begin {document}" << std::endl;
        oss << "\\paragraph{Edges}" << std::endl;
        oss << "\\begin{enumerate}" << std::endl;
        for (auto edge : transitions) {
          oss << "\\item \\texttt{" << edge->name() << "}" << std::endl;
        }
        oss << "\\end{enumerate}" << std::endl;
        oss << "\\begin {tabular}{l";
        for (size_type j=0; j<m.cols (); ++j)
          oss << "c";
        oss << "}" << std::endl;
        oss << "Solver index";
        for (size_type j=0; j<m.cols (); ++j)
          oss << " & " << j+1;
        oss << "\\\\" << std::endl;
        for (size_type i=0; i<m.rows (); ++i) {
          oss << "\\texttt{" << constraints [i]->function ().name () << "} & " << std::endl;
          for (size_type j=0; j<m.cols (); ++j) {
            oss << m (i,j);
            if (j < m.cols () - 1)
              oss << " & ";
          }
          oss << "\\\\" << std::endl;
        }
        oss << "\\end{tabular}" << std::endl;
        oss << "\\end {document}" << std::endl;

        std::string s = oss.str ();
        std::string s2 = "";
        for (int i=0; i < s.size(); i++) {
          if (s[i] == '_') s2 += "\\_";
          else s2.push_back(s[i]);
        }
        hppDout (info, s2);
      }

      bool CrossStateOptimization::contains
      (const Solver_t& solver, const ImplicitPtr_t& c) const
      {
        if (solver.contains (c)) return true;
        std::map <ImplicitPtr_t, ImplicitPtr_t>::const_iterator it
          (sameRightHandSide_.find (c));
        if ((it != sameRightHandSide_.end () &&
             solver.contains (it->second))) {
          return true;
        }
        return false;
      }

      bool CrossStateOptimization::buildOptimizationProblem
      (OptimizationData& d, const graph::Edges_t& transitions) const
      {
        if (d.N == 0) return true; // TODO: false when there is only a "loop | f"
        d.M_status.resize (constraints_.size (), d.N);
        d.M_status.fill (OptimizationData::ABSENT);
        d.M_rhs.resize (constraints_.size (), d.N);
        d.M_rhs.fill (LiegroupElement ());
        size_type index = 0;
        // Loop over constraints
        for (NumericalConstraints_t::const_iterator it (constraints_.begin ());
             it != constraints_.end (); ++it) {
          const ImplicitPtr_t& c (*it);
          // Loop forward over waypoints to determine right hand sides equal
          // to initial configuration
          for (std::size_t j = 0; j < d.N; ++j) {
            // Get transition solver
            const Solver_t& trSolver
              (transitions [j]->pathConstraint ()->configProjector ()->
               solver ());
            if (contains (trSolver, c)) {
              if ((j==0) || d.M_status (index, j-1) ==
                  OptimizationData::EQUAL_TO_INIT) {
                d.M_status (index, j) = OptimizationData::EQUAL_TO_INIT;
              } else {
                d.M_status (index, j) = OptimizationData::EQUAL_TO_PREVIOUS;
              }
            }
          }
          // Loop backward over waypoints to determine right hand sides equal
          // to final configuration
          for (size_type j = d.N-1; j > 0; --j) {
            // Get transition solver
            const Solver_t& trSolver
              (transitions [(std::size_t)j+1]->pathConstraint ()->
               configProjector ()->solver ());
            if (contains (trSolver, c)) {
              if ((j==(size_type) d.N-1) || d.M_status (index, j+1) ==
                  OptimizationData::EQUAL_TO_GOAL) {
                // If constraint right hand side is already equal to
                // initial config, check that right hand side is equal
                // for init and goal configs.
                if (d.M_status (index, j) ==
                    OptimizationData::EQUAL_TO_INIT) {
                  if (checkConstantRightHandSide (d, index)) {
                    // stop for this constraint
                    break;
                  } else {
                    // Right hand side of constraint should be equal along the
                    // whole path but is different at init and at goal configs.
                    return false;
                  }
                } else {
                  d.M_status (index, j) = OptimizationData::EQUAL_TO_GOAL;
                }
              }
            }
          }
          ++index;
        } // for (NumericalConstraints_t::const_iterator it
        displayStatusMatrix (d.M_status, constraints_, transitions);
        graph::GraphPtr_t cg (problem_->constraintGraph ());
        // Fill solvers with target constraints of transition
        for (std::size_t j = 0; j < d.N; ++j) {
          d.solvers [(std::size_t)j] = transitions [(std::size_t)j]->
            targetConstraint()->configProjector ()->solver ();
        }
        // Initial guess
        std::vector<size_type> ks;
        size_type K = 0;
        ks.resize(d.N);
        for (std::size_t i = 0; i < d.N + 1; ++i) {
          if (!transitions[i]->isShort()) ++K;
          if (i < d.N) ks[i] = K;
        }
        if (K==0) {
          ++K;
          for (std::size_t i = d.N/2; i < d.N; ++i)
            ks[i] = 1;
        }
        for (std::size_t i = 0; i < d.N; ++i) {
          value_type u = value_type(ks[i]) / value_type(K);
          pinocchio::interpolate (d.robot, d.q1, d.q2, u, d.qInit.col (i));
          hppDout (info, "qInit = " << pinocchio::displayConfig
                   (d.qInit.col (i)));
        }

        return true;
      }

      bool CrossStateOptimization::solveOptimizationProblem
      (OptimizationData& d) const
      {
        // Iterate on waypoint solvers, for each of them
        //  1. initialize right hand side,
        //  2. solve.
        for (std::size_t j=0; j<d.solvers.size (); ++j) {
          Solver_t& solver (d.solvers [j]);
          for (std::size_t i=0; i<constraints_.size (); ++i) {
            const ImplicitPtr_t& c (constraints_ [i]);
              switch (d.M_status ((size_type)i, (size_type)j)) {
            case OptimizationData::EQUAL_TO_PREVIOUS:
              assert (j != 0);
              solver.rightHandSideFromConfig (c, d.waypoint.col (j-1));
              break;
            case OptimizationData::EQUAL_TO_INIT:
              solver.rightHandSideFromConfig (c, d.q1);
              break;
            case OptimizationData::EQUAL_TO_GOAL:
              solver.rightHandSideFromConfig (c, d.q2);
              break;
            case OptimizationData::ABSENT:
            default:
              ;
            }
          }
          if (j == 0)
            d.waypoint.col (j) = d.qInit.col (j);
          else
            d.waypoint.col (j) = d.waypoint.col (j-1);
          Solver_t::Status status = solver.solve
            (d.waypoint.col (j),
             constraints::solver::lineSearch::Backtracking ());
          size_type nbTry=0;
          size_type nRandomConfigs(problem()->getParameter
                         ("CrossStateOptimization/nRandomConfigs").intValue());

          while(status != Solver_t::SUCCESS && nbTry < nRandomConfigs){
            d.waypoint.col (j) = *(problem()->configurationShooter()->shoot());
            status = solver.solve (d.waypoint.col (j),
              constraints::solver::lineSearch::Backtracking ());
            ++nbTry;
          }
          switch (status) {
          case Solver_t::ERROR_INCREASED:
            hppDout (info, "  error increased at step " << j);
            return false;
          case Solver_t::MAX_ITERATION_REACHED:
            hppDout (info, "  max iteration reached at step " << j);
            return false;
          case Solver_t::INFEASIBLE:
            hppDout (info, "  infeasible at step " << j);
            return false;
          case Solver_t::SUCCESS:
            hppDout(info,  "  config solved at transition " << j << ": " << pinocchio::displayConfig(d.waypoint.col(j)));
            ;
          }
        }
        hppDout (info, "  success");
        return true;
      }

      core::PathVectorPtr_t CrossStateOptimization::buildPath (
          OptimizationData& d, const Edges_t& transitions) const
      {
        using core::PathVector;
        using core::PathVectorPtr_t;

        const core::DevicePtr_t& robot = problem()->robot();
        PathVectorPtr_t pv = PathVector::create (
            robot->configSize(), robot->numberDof());
        core::PathPtr_t path;

        std::size_t i = 0;
        for (Edges_t::const_iterator _t = transitions.begin();
            _t != transitions.end(); ++_t)
        {
          const EdgePtr_t& t = *_t;
          bool first = (i == 0);
          bool last  = (i == d.N);

          bool status;
          if (first && last)
            status = t->build (path, d.q1, d.q2);
          else if (first)
            status = t->build (path, d.q1, d.waypoint.col (i));
          else if (last)
            status = t->build (path, d.waypoint.col (i-1), d.q2);
          else {
            status = t->build (path, d.waypoint.col (i-1), d.waypoint.col (i));
          }
          // This might fail when last argument constraint error is slightly above the threshold
          if (!status || !path) {
            return PathVectorPtr_t();
          }
          pv->appendPath(path);

          ++i;
        }
        return pv;
      }

      core::PathPtr_t CrossStateOptimization::impl_compute (
          ConfigurationIn_t q1, ConfigurationIn_t q2) const
      {
        const graph::GraphPtr_t& graph(problem_->constraintGraph ());
        GraphSearchData d;
        d.s1 = graph->getState (q1);
        d.s2 = graph->getState (q2);

        d.maxDepth = problem_->getParameter
	        ("CrossStateOptimization/maxDepth").intValue();
        int cnt = 0;
        std::size_t nTriesForEachPath = problem_->getParameter
	        ("CrossStateOptimization/nTriesForEachPath").intValue();

        // Find
        d.queue1.push (d.addInitState());
        std::size_t idxSol = (d.s1 == d.s2 ? 1 : 0);

        bool maxDepthReached;
        while (!(maxDepthReached = findTransitions (d))) { // mut
#ifdef LSE_GET_TRANSITION_LISTS
          std::vector<Edges_t> transitionss = getTransitionLists (d, idxSol); // const, const
          cnt += transitionss.size();
          while (! transitionss.empty()) {
            for (std::size_t nTry = 0; nTry < nTriesForEachPath; nTry++) {
              for (std::size_t idySol = 0; idySol < transitionss.size(); idySol++) {
                const Edges_t& transitions = transitionss[idySol];
#else
          Edges_t transitions = getTransitionList (d, idxSol); // const, const
          while (! transitions.empty()) {
            for (std::size_t nTry = 0; nTry < nTriesForEachPath; nTry++) {
                std::size_t idySol = 0;
#endif
#ifdef HPP_DEBUG
                std::ostringstream ss;
                ss << " Trying solution " << idxSol << "-" << idySol <<
                      ", try " << nTry << ": \n\t";
                for (std::size_t j = 0; j < transitions.size(); ++j)
                  ss << transitions[j]->name() << ", \n\t";
                hppDout (info, ss.str());
#endif // HPP_DEBUG
                OptimizationData optData (problem(), q1, q2, transitions);
                if (buildOptimizationProblem (optData, transitions)) {
                  if (solveOptimizationProblem (optData)) {
                    core::PathPtr_t path = buildPath (optData, transitions);
                    if (path) {
                      hppDout (info, " Success for solution " << idxSol <<
                        "-" << idySol << ", return path, try" << nTry+1);
                      return path; // comment to see other transitions which would have worked
                      idySol = SIZE_MAX-1;
                      nTry = SIZE_MAX-1;
                      // we already know this path works so let's move on to the next
                    } else {
                      hppDout (info, " Failed solution " << idxSol <<
                        "-" << idySol << " at step 5 (build path)");
                    }
                  } else {
                    hppDout (info, " Failed solution " << idxSol <<
                      "-" << idySol << " at step 4 (solve opt pb)");
                  }
                } else {
                  hppDout (info, " Failed solution " << idxSol <<
                    "-" << idySol << " at step 3 (build opt pb)");
                  idySol = SIZE_MAX-1;
                  nTry = SIZE_MAX-1;
                  // no other LSE alter ego will go further than step 3
                }
              }
#ifdef LSE_GET_TRANSITION_LISTS
            }
            ++idxSol;
            transitionss = getTransitionLists(d, idxSol);
#else
            ++idxSol;
            transitions = getTransitionList(d, idxSol);
#endif
          }
        }
        hppDout (warning, " Max depth reached");
#ifdef LSE_GET_TRANSITION_LISTS
        hppDout (warning, cnt << " transitions in total");
#endif

        return core::PathPtr_t ();
      }

      using core::Parameter;
      using core::ParameterDescription;

      HPP_START_PARAMETER_DECLARATION(CrossStateOptimization)
      core::Problem::declareParameter(ParameterDescription(Parameter::INT,
            "CrossStateOptimization/maxDepth",
            "Maximum number of transitions to look for.",
            Parameter((size_type)2)));
      core::Problem::declareParameter(ParameterDescription(Parameter::INT,
            "CrossStateOptimization/maxIteration",
            "Maximum number of iterations of the Newton Raphson algorithm.",
            Parameter((size_type)60)));
      core::Problem::declareParameter(ParameterDescription(Parameter::FLOAT,
            "CrossStateOptimization/errorThreshold",
            "Error threshold of the Newton Raphson algorithm.",
            Parameter(1e-4)));
      core::Problem::declareParameter(ParameterDescription(Parameter::INT,
            "CrossStateOptimization/nRandomConfigs",
            "Number of random configurations to sample to initialize each "
            "solver.", Parameter((size_type)0)));
      core::Problem::declareParameter(ParameterDescription(Parameter::INT,
            "CrossStateOptimization/nTriesForEachPath",
            "Number of tries to be done for each state path "
            "solver.", Parameter((size_type)1)));
      HPP_END_PARAMETER_DECLARATION(CrossStateOptimization)
    } // namespace steeringMethod
  } // namespace manipulation
} // namespace hpp
