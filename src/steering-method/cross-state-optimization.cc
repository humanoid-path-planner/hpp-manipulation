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

#include <boost/bind.hpp>

#include <hpp/util/exception-factory.hh>

#include <pinocchio/multibody/model.hpp>

#include <hpp/pinocchio/configuration.hh>

#include <hpp/constraints/hybrid-solver.hh>

#include <hpp/core/explicit-numerical-constraint.hh>
#include <hpp/core/path-vector.hh>

#include <hpp/manipulation/graph/edge.hh>
#include <hpp/manipulation/graph/state.hh>

#include <../src/steering-method/cross-state-optimization/function.cc>

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
          const Problem& problem)
      {
        CrossStateOptimizationPtr_t shPtr (new CrossStateOptimization (problem));
        shPtr->init(shPtr);
        return shPtr;
      }

      CrossStateOptimizationPtr_t CrossStateOptimization::create (
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
            : s(_e->from()), e(_e), l(_l), i (_i) {}
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
                transition->to(), state_with_depths_t ()
                )).first;

          next->second.push_back (
              state_with_depth(transition, from.l + 1, _p.parentIdx));

          return state_with_depth_ptr_t (next, next->second.size()-1);
        }
      };

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

            done = done || (transition->to() == d.s2);
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

      struct CrossStateOptimization::OptimizationData
      {
        const std::size_t N, nq, nv;
        constraints::HybridSolver solver;
        Configuration_t q1, q2;
        vector_t q;
        core::DevicePtr_t robot;
        typedef std::vector<States_t> StatesPerConf_t;
        StatesPerConf_t statesPerConf_;
        struct RightHandSideSetter {
          DifferentiableFunctionPtr_t impF;
          size_type expFidx;
          Configuration_t* qrhs;
          vector_t rhs;
          RightHandSideSetter () : qrhs (NULL) {}
          // TODO delete this constructor
          RightHandSideSetter (DifferentiableFunctionPtr_t _impF, size_type _expFidx, Configuration_t* _qrhs)
            : impF(_impF), expFidx(_expFidx), qrhs (_qrhs) {}
          RightHandSideSetter (DifferentiableFunctionPtr_t _impF, size_type _expFidx, vector_t _rhs)
            : impF(_impF), expFidx(_expFidx), qrhs (NULL), rhs (_rhs) {}
          void apply(constraints::HybridSolver& s)
          {
            if (expFidx >= 0) {
              if (qrhs != NULL) s.explicitSolver().rightHandSideFromInput (expFidx, *qrhs);
              else              s.explicitSolver().rightHandSide          (expFidx, rhs);
            } else {
              if (qrhs != NULL) s.rightHandSideFromInput (impF, DifferentiableFunctionPtr_t(), *qrhs);
              else              s.rightHandSide          (impF, DifferentiableFunctionPtr_t(), rhs);
            }
          }
        };
        typedef std::vector<RightHandSideSetter> RightHandSideSetters_t;
        RightHandSideSetters_t rhsSetters_;

        OptimizationData (const std::size_t& _N, const core::DevicePtr_t _robot)
          : N (_N), nq (_robot->configSize()), nv (_robot->numberDof()),
          solver (N * nq, N * nv), robot (_robot), statesPerConf_ (N)
        {
          solver.integration (boost::bind(&OptimizationData::_integrate, this, _1, _2, _3));
          solver.saturation  (boost::bind(&OptimizationData::_saturate , this, _1, _2));
        }

        void addGraphConstraints (const graph::GraphPtr_t& graph)
        {
          for (std::size_t i = 0; i < N; ++i) {
            _add (graph->lockedJoints(), i);
            _add (graph->numericalConstraints(), i);
          }
        }

        void addConstraints (const StatePtr_t& state, const std::size_t& i)
        {
          bool alreadyAdded = (
              std::find (statesPerConf_[i].begin(), statesPerConf_[i].end(), state)
              != statesPerConf_[i].end());
          if (alreadyAdded) return;
          _add (state->lockedJoints(), i);
          _add (state->numericalConstraints(), i);
          statesPerConf_[i].push_back(state);
        }

        void addConstraints (const EdgePtr_t& trans, const std::size_t& i)
        {
          const LockedJoints_t& ljs = trans->lockedJoints();
          for (LockedJoints_t::const_iterator _lj = ljs.begin();
              _lj != ljs.end(); ++_lj) {
            LockedJointPtr_t lj (*_lj);
            std::ostringstream os;
            os << lj->jointName() << " | " << i << " -> " << (i+1);
            DifferentiableFunctionPtr_t f, f_implicit;
            // i = Input, o = Output,
            // c = Config, v = Velocity
            RowBlockIndices ic, oc, ov;
            ColBlockIndices iv;
            ComparisonTypes_t cts;
            vector_t rhs;
            if (i == 0) {
              f = lj->explicitFunction();
              ic = _row(lj->inputConf()     , 0);
              oc = _row(lj->outputConf()    , 0);
              iv = _col(lj->inputVelocity() , 0);
              ov = _row(lj->outputVelocity(), 0);
              cts = lj->comparisonType();
              lj->rightHandSideFromConfig (q1);
              rhs = lj->rightHandSide();
            } else if (i == N) {
              f = lj->explicitFunction();
              // Currently, this function is mandatory because if the same
              // joint is locked all along the path, then, one of the LockedJoint 
              // has to be treated implicitely.
              // TODO it would be smarter to detect this case beforehand. If the
              // chain in broken in the middle, an explicit formulation exists
              // (by inverting the equality in the next else section) and we
              // miss it.
              f_implicit = _stateFunction (lj->functionPtr(), N-1);
              ic = _row(lj->inputConf()     , 0);
              oc = _row(lj->outputConf()    , (N-1) * nq);
              iv = _col(lj->inputVelocity() , 0);
              ov = _row(lj->outputVelocity(), (N-1) * nv);
              cts = lj->comparisonType();
              lj->rightHandSideFromConfig (q2);
              rhs = lj->rightHandSide();
            } else {
              f = Identity::Ptr_t (new Identity (lj->configSpace(), os.str()));
              ic = _row(lj->outputConf()    , (i - 1) * nq);
              oc = _row(lj->outputConf()    ,  i      * nq);
              iv = _col(lj->outputVelocity(), (i - 1) * nv);
              ov = _row(lj->outputVelocity(),  i      * nv);
              cts = ComparisonTypes_t (lj->numberDof(), constraints::EqualToZero);
            }

            // It is important to use the index of the function since the same
            // function may be added several times on different part.
            size_type expFidx = solver.explicitSolver().add (f, ic, oc, iv, ov, cts);
            if (expFidx < 0) {
              if (f_implicit) {
                solver.add (f_implicit, 0, cts);
              } else {
                HPP_THROW (std::invalid_argument,
                    "Could not add locked joint " << lj->jointName() <<
                    " of transition " << trans->name() << " at id " << i);
              }
            }

            // Setting the right hand side must be done later
            if (rhs.size() > 0)
              rhsSetters_.push_back (RightHandSideSetter(
                    f_implicit, expFidx, rhs));
            f_implicit.reset();
          }

          // TODO handle numerical constraints
          using namespace ::hpp::core;
          ExplicitNumericalConstraintPtr_t enc;
          const NumericalConstraints_t& ncs = trans->numericalConstraints();
          for (NumericalConstraints_t::const_iterator _nc = ncs.begin();
              _nc != ncs.end(); ++_nc) {
            NumericalConstraintPtr_t nc (*_nc);
            enc = HPP_DYNAMIC_PTR_CAST (ExplicitNumericalConstraint, nc);

            DifferentiableFunctionPtr_t f, ef;
            // i = Input, o = Output,
            // c = Config, v = Velocity
            // Configuration_t* qrhs;
            RowBlockIndices ic, oc, ov;
            ColBlockIndices iv;
            ComparisonTypes_t cts;
            vector_t rhs;
            if (i == 0) {
              f = _stateFunction(nc->functionPtr(), 0);
              // if (enc) {
                // ef = enc->explicitFunction();
                // ic = _row(enc->inputConf()     , 0);
                // oc = _row(enc->outputConf()    , 0);
                // iv = _col(enc->inputVelocity() , 0);
                // ov = _row(enc->outputVelocity(), 0);
              // }
              // cts = ComparisonTypes_t (enc->outputDerivativeSize(), constraints::Equality);
              cts = nc->comparisonType ();
              nc->rightHandSideFromConfig (q1);
              rhs = nc->rightHandSide();
              // qrhs = &q1;
            } else if (i == N) {
              f = _stateFunction(nc->functionPtr(), N - 1);
              cts = nc->comparisonType ();
              nc->rightHandSideFromConfig (q2);
              rhs = nc->rightHandSide();
              // qrhs = &q2;
            } else {
              f = _edgeFunction (nc->functionPtr(), i - 1, i);
              cts = ComparisonTypes_t (f->outputSize(), constraints::EqualToZero);
              // qrhs = NULL;
            }

            size_type expFidx = -1;
            if (ef) expFidx = solver.explicitSolver().add (ef, ic, oc, iv, ov, cts);
            if (expFidx < 0) solver.add (f, 0, cts);

            // TODO This must be done later...
            // if (qrhs != NULL) {
            if (rhs.size() > 0) {
              // solver.rightHandSideFromInput (f, ef, rhs);
              rhsSetters_.push_back (RightHandSideSetter(f, expFidx, rhs));
            }
          }
        }

        void setRightHandSide ()
        {
          for (std::size_t i = 0; i < rhsSetters_.size(); ++i)
            rhsSetters_[i].apply(solver);
        }

        bool solve ()
        {
          if (N == 0) return true;
          constraints::HierarchicalIterativeSolver::Status status =
            solver.solve <constraints::lineSearch::FixedSequence> (q);
          bool success = (status == constraints::HierarchicalIterativeSolver::SUCCESS);
          if (!success) {
            hppDout (warning, "Projection failed with status " << status <<
                ". Configuration after projection is\n"
                << pinocchio::displayConfig(q));
          }
          return success;
        }

        void _add (const NumericalConstraints_t& ncs, const std::size_t& i)
        {
          using namespace ::hpp::core;
          ExplicitNumericalConstraintPtr_t enc;
          for (NumericalConstraints_t::const_iterator _nc = ncs.begin();
              _nc != ncs.end(); ++_nc) {
            NumericalConstraintPtr_t nc (*_nc);
            enc = HPP_DYNAMIC_PTR_CAST (ExplicitNumericalConstraint, nc);
            bool added = false;
            if (enc) {
              added = solver.explicitSolver().add (enc->explicitFunction(),
                  _row(enc->inputConf()     , i * nq),
                  _row(enc->outputConf()    , i * nq),
                  _col(enc->inputVelocity() , i * nv),
                  _row(enc->outputVelocity(), i * nv),
                  enc->comparisonType ());
            }
            if (!added)
              solver.add (_stateFunction(nc->functionPtr(), i), 0, nc->comparisonType());
          }
        }
        void _add (const LockedJoints_t& ljs, const std::size_t i)
        {
          for (LockedJoints_t::const_iterator _lj = ljs.begin();
              _lj != ljs.end(); ++_lj) {
            LockedJointPtr_t lj (*_lj);
            size_type expFidx = solver.explicitSolver().add (
                lj->explicitFunction(),
                _row(lj->inputConf()     , i * nq),
                _row(lj->outputConf()    , i * nq),
                _col(lj->inputVelocity() , i * nv),
                _row(lj->outputVelocity(), i * nv),
                lj->comparisonType ());
            if (expFidx < 0)
              throw std::invalid_argument ("Could not add locked joint " + lj->jointName());

            // This must be done later
            rhsSetters_.push_back (RightHandSideSetter(
                  DifferentiableFunctionPtr_t(),
                  expFidx,
                  lj->rightHandSide()));
            // solver.rightHandSide (DifferentiableFunctionPtr_t(),
                // lj->explicitFunction(),
                // lj->rightHandSide());
          }
        }

        RowBlockIndices _row (segments_t s, const std::size_t& shift)
        {
          for (std::size_t j = 0; j < s.size(); ++j) s[j].first += shift;
          return RowBlockIndices (s);
        }
        ColBlockIndices _col (segments_t s, const std::size_t& shift)
        {
          for (std::size_t j = 0; j < s.size(); ++j) s[j].first += shift;
          return ColBlockIndices (s);
        }
        StateFunction::Ptr_t _stateFunction(const DifferentiableFunctionPtr_t inner, const std::size_t i)
        {
          assert (i < N);
          return StateFunction::Ptr_t (new StateFunction (inner, N * nq, N * nv,
                segment_t (i * nq, nq), segment_t (i * nv, nv)
                ));
        }
        EdgeFunction::Ptr_t _edgeFunction(const DifferentiableFunctionPtr_t inner,
            const std::size_t iL, const std::size_t iR)
        {
          assert (iL < N && iR < N);
          return EdgeFunction::Ptr_t (new EdgeFunction (inner, N * nq, N * nv,
                segment_t (iL * nq, nq), segment_t (iL * nv, nv),
                segment_t (iR * nq, nq), segment_t (iR * nv, nv)));
        }
        void _integrate (vectorIn_t qin, vectorIn_t v, vectorOut_t qout)
        {
          for (std::size_t i = 0, iq = 0, iv = 0; i < N; ++i, iq += nq, iv += nv)
            pinocchio::integrate (robot,
                qin.segment(iq,nq),
                v.segment(iv,nv),
                qout.segment(iq,nq));
        }
        bool _saturate (vectorIn_t q, Eigen::VectorXi& sat)
        {
          bool ret = false;
          const se3::Model& model = robot->model();

          for (std::size_t i = 1; i < model.joints.size(); ++i) {
            const size_type jnq = model.joints[i].nq();
            const size_type jnv = model.joints[i].nv();
            const size_type jiq = model.joints[i].idx_q();
            const size_type jiv = model.joints[i].idx_v();
            for (std::size_t k = 0; k < N; ++k) {
              const size_type idx_q = k * nq + jiq;
              const size_type idx_v = k * nv + jiv;
              for (size_type j = 0; j < jnq; ++j) {
                const size_type iq = idx_q + j;
                const size_type iv = idx_v + std::min(j,jnv-1);
                if        (q[iq] >= model.upperPositionLimit[jiq + j]) {
                  sat[iv] =  1;
                  ret = true;
                } else if (q[iq] <= model.lowerPositionLimit[jiq + j]) {
                  sat[iv] = -1;
                  ret = true;
                } else
                  sat[iv] =  0;
              }
            }
          }

          const hpp::pinocchio::ExtraConfigSpace& ecs = robot->extraConfigSpace();
          const size_type& d = ecs.dimension();

          for (size_type k = 0; k < d; ++k) {
            for (std::size_t j = 0; j < N; ++j) {
              const size_type iq = j * nq + model.nq + k;
              const size_type iv = j * nv + model.nv + k;
              if        (q[iq] >= ecs.upper(k)) {
                sat[iv] =  1;
                ret = true;
              } else if (q[iq] <= ecs.lower(k)) {
                sat[iv] = -1;
                ret = true;
              } else
                sat[iv] =  0;
            }
          }
          return ret;
        }
      };

      void CrossStateOptimization::buildOptimizationProblem (
          OptimizationData& d, const Edges_t& transitions) const
      {
        if (d.N == 0) return;
        size_type maxIter = problem_.getParameter ("CrossStateOptimization/maxIteration").intValue();
        value_type thr = problem_.getParameter ("CrossStateOptimization/errorThreshold").floatValue();
        d.solver.maxIterations (maxIter);
        d.solver.errorThreshold (thr);

        // Add graph constraints      (decoupled)
        d.addGraphConstraints (problem_.constraintGraph());

        std::size_t i = 0;
        for (Edges_t::const_iterator _t = transitions.begin();
            _t != transitions.end(); ++_t)
        {
          const EdgePtr_t& t = *_t;
          bool first = (i == 0);
          bool last  = (i == d.N);

          // Add state constraints      (decoupled)
          if (!last ) d.addConstraints (t->to()   , i);
          if (!last ) d.addConstraints (t->state(), i);
          if (!first) d.addConstraints (t->from() , i - 1);
          if (!first) d.addConstraints (t->state(), i - 1);

          // Add transition constraints (coupled)
          d.addConstraints (t, i);

          ++i;
        }

        d.solver.explicitSolverHasChanged();
        d.setRightHandSide();

        hppDout (info, "Solver informations\n" << d.solver);

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
        d.q.resize (d.N * d.nq);
        for (std::size_t i = 0; i < d.N; ++i) {
          value_type u = value_type(ks[i]) / value_type(K);
          pinocchio::interpolate (d.robot, d.q1, d.q2, u, d.q.segment (i * d.nq, d.nq));
        }
      }

      core::PathVectorPtr_t CrossStateOptimization::buildPath (
          OptimizationData& d, const Edges_t& transitions) const
      {
        using core::PathVector;
        using core::PathVectorPtr_t;

        const core::DevicePtr_t& robot = problem().robot();
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
            status = t->build (path, d.q1, d.q.head (d.nq));
          else if (last)
            status = t->build (path, d.q.tail (d.nq), d.q2);
          else {
            std::size_t j = (i-1) * d.nq;
            status = t->build (path, d.q.segment (j, d.nq), d.q.segment (j + d.nq, d.nq));
          }

          if (!status || !path) {
            hppDout (warning, "Could not build path from solution "
                << pinocchio::displayConfig(d.q));
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
        const graph::Graph& graph = *problem_.constraintGraph ();
        GraphSearchData d;
        d.s1 = graph.getState (q1);
        d.s2 = graph.getState (q2);
        // d.maxDepth = 2;
        d.maxDepth = problem_.getParameter ("CrossStateOptimization/maxDepth").intValue();

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

            OptimizationData optData (transitions.size() - 1, problem().robot());
            optData.q1 = q1;
            optData.q2 = q2;
            bool ok = true;
            try {
              buildOptimizationProblem (optData, transitions);
            } catch (const std::invalid_argument& e) {
              hppDout (info, e.what ());
              ok = false;
            }

            if (ok && optData.solve()) {
              core::PathPtr_t path = buildPath (optData, transitions);
              if (path) return path;
              hppDout (info, "Failed to build path from solution: "
                    << pinocchio::displayConfig(optData.q));
            } else {
              hppDout (info, "Failed to solve");
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
      HPP_END_PARAMETER_DECLARATION(CrossStateOptimization)
    } // namespace steeringMethod
  } // namespace manipulation
} // namespace hpp
