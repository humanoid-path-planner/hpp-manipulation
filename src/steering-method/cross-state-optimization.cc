// Copyright (c) 2017, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr),
//          Florent Lamiraux (florent.lamiraux@laas.fr)
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
          GraphSearchData& d, const std::size_t& i) const
      {
        assert (d.parent1.find (d.s2) != d.parent1.end());
        const GraphSearchData::state_with_depths_t& roots = d.parent1[d.s2];
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
          current = &d.parent1[current->s][current->i];
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
          oss << "\\item " << edge->name() << std::endl;
        }
        oss << "\\end{enumerate}" << std::endl;
        oss << "\\begin {tabular}{";
        for (size_type j=0; j<m.cols () + 1; ++j)
          oss << "c";
        oss << "}" << std::endl;
        for (size_type i=0; i<m.rows (); ++i) {
          oss << constraints [i]->function ().name () << " & ";
          for (size_type j=0; j<m.cols (); ++j) {
            oss << m (i,j);
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
        if (d.N == 0) return true;
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
            status = solver.solve
            (d.waypoint.col (j),
             constraints::solver::lineSearch::Backtracking ());
            ++nbTry;
          }
          switch (status) {
          case Solver_t::ERROR_INCREASED:
            hppDout (info, "error increased.");
            return false;
          case Solver_t::MAX_ITERATION_REACHED:
            hppDout (info, "max iteration reached.");
            return false;
          case Solver_t::INFEASIBLE:
            hppDout (info, "infeasible.");
            return false;
          case Solver_t::SUCCESS:
            hppDout (info, "success.");
          }
        }
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

          if (!status || !path) {
            hppDout (warning, "Could not build path from solution ");
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
        // d.maxDepth = 2;
        d.maxDepth = problem_->getParameter
	  ("CrossStateOptimization/maxDepth").intValue();

        // Find
        d.queue1.push (d.addInitState());
        std::size_t idxSol = (d.s1 == d.s2 ? 1 : 0);
        bool maxDepthReached = findTransitions (d);

        while (!maxDepthReached) {
          Edges_t transitions = getTransitionList (d, idxSol);
          while (! transitions.empty()) {
#ifdef HPP_DEBUG
            std::ostringstream ss;
            ss << "Trying solution " << idxSol << ": ";
            for (std::size_t j = 0; j < transitions.size(); ++j)
              ss << transitions[j]->name() << ", ";
            hppDout (info, ss.str());
#endif // HPP_DEBUG

            OptimizationData optData (problem(), q1, q2, transitions);
            if (buildOptimizationProblem (optData, transitions)) {
              if (solveOptimizationProblem (optData)) {
                core::PathPtr_t path = buildPath (optData, transitions);
                if (path) return path;
                hppDout (info, "Failed to build path from solution: ");
              } else {
                hppDout (info, "Failed to solve");
              }
            }
            ++idxSol;
            transitions = getTransitionList(d, idxSol);
          }
          maxDepthReached = findTransitions (d);
        }

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
      HPP_END_PARAMETER_DECLARATION(CrossStateOptimization)
    } // namespace steeringMethod
  } // namespace manipulation
} // namespace hpp
