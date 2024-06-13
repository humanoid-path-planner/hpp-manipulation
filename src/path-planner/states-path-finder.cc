// Copyright (c) 2021, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr),
//          Florent Lamiraux (florent.lamiraux@laas.fr)
//          Alexandre Thiault (athiault@laas.fr)
//          Le Quang Anh (quang-anh.le@laas.fr)
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

#include <hpp/constraints/affine-function.hh>
#include <hpp/constraints/explicit.hh>
#include <hpp/constraints/locked-joint.hh>
#include <hpp/constraints/solver/by-substitution.hh>
#include <hpp/core/collision-validation-report.hh>
#include <hpp/core/collision-validation.hh>
#include <hpp/core/configuration-shooter.hh>
#include <hpp/core/diffusing-planner.hh>
#include <hpp/core/path-optimization/random-shortcut.hh>
#include <hpp/core/path-planning-failed.hh>
#include <hpp/core/path-projector/progressive.hh>
#include <hpp/core/path-validation.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/core/problem-target/goal-configurations.hh>
#include <hpp/core/problem-target/task-target.hh>
#include <hpp/manipulation/constraint-set.hh>
#include <hpp/manipulation/graph/edge.hh>
#include <hpp/manipulation/graph/state-selector.hh>
#include <hpp/manipulation/graph/state.hh>
#include <hpp/manipulation/path-planner/states-path-finder.hh>
#include <hpp/manipulation/roadmap.hh>
#include <hpp/pinocchio/configuration.hh>
#include <hpp/pinocchio/joint-collection.hh>
#include <hpp/util/debug.hh>
#include <hpp/util/exception-factory.hh>
#include <hpp/util/timer.hh>
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <algorithm>
#include <iomanip>
#include <map>
#include <typeinfo>
#include <vector>

namespace hpp {
namespace manipulation {
namespace pathPlanner {

using Eigen::ColBlockIndices;
using Eigen::RowBlockIndices;

using graph::EdgePtr_t;
using graph::Edges_t;
using graph::LockedJoints_t;
using graph::Neighbors_t;
using graph::NumericalConstraints_t;
using graph::segments_t;
using graph::StatePtr_t;
using graph::States_t;

using core::ProblemTargetPtr_t;
using core::problemTarget::GoalConfigurations;
using core::problemTarget::GoalConfigurationsPtr_t;
using core::problemTarget::TaskTarget;
using core::problemTarget::TaskTargetPtr_t;

#ifdef HPP_DEBUG
static void displayRoadmap(const core::RoadmapPtr_t& roadmap) {
  unsigned i = 0;
  for (auto cc : roadmap->connectedComponents()) {
    hppDout(info, " CC " << i++);
    for (auto n : cc->nodes()) {
      hppDout(info, pinocchio::displayConfig(n->configuration()));
    }
  }
}
#endif

StatesPathFinder::StatesPathFinder(const core::ProblemConstPtr_t& problem,
                                   const core::RoadmapPtr_t& roadmap)
    : PathPlanner(problem, roadmap),
      problem_(HPP_STATIC_PTR_CAST(const manipulation::Problem, problem)),
      constraints_(),
      index_(),
      sameRightHandSide_(),
      stricterConstraints_(),
      optData_(0x0),
      graphData_(0x0),
      lastBuiltTransitions_(),
      goalConstraints_(),
      goalDefinedByConstraints_(false),
      q1_(),
      q2_(),
      configList_(),
      idxConfigList_(0),
      nTryConfigList_(0),
      solved_(false),
      interrupt_(false),
      weak_() {
  gatherGraphConstraints();
  inStateProblem_ = core::Problem::create(problem_->robot());
}

StatesPathFinder::StatesPathFinder(const StatesPathFinder& other)
    : PathPlanner(other.problem_),
      problem_(other.problem_),
      constraints_(),
      index_(other.index_),
      sameRightHandSide_(other.sameRightHandSide_),
      weak_() {}

StatesPathFinderPtr_t StatesPathFinder::create(
    const core::ProblemConstPtr_t& problem) {
  StatesPathFinder* ptr;
  RoadmapPtr_t r = Roadmap::create(problem->distance(), problem->robot());
  try {
    ProblemConstPtr_t p(HPP_DYNAMIC_PTR_CAST(const Problem, problem));
    ptr = new StatesPathFinder(p, r);
  } catch (std::exception&) {
    throw std::invalid_argument(
        "The problem must be of type hpp::manipulation::Problem.");
  }
  StatesPathFinderPtr_t shPtr(ptr);
  ptr->init(shPtr);
  return shPtr;
}

StatesPathFinderPtr_t StatesPathFinder::createWithRoadmap(
    const core::ProblemConstPtr_t& problem, const core::RoadmapPtr_t& roadmap) {
  StatesPathFinder* ptr;
  ProblemConstPtr_t p = HPP_DYNAMIC_PTR_CAST(const Problem, problem);
  RoadmapPtr_t r = HPP_DYNAMIC_PTR_CAST(Roadmap, roadmap);
  if (!r)
    throw std::invalid_argument(
        "The roadmap must be of type hpp::manipulation::Roadmap.");
  if (!p)
    throw std::invalid_argument(
        "The problem must be of type hpp::manipulation::Problem.");

  ptr = new StatesPathFinder(p, r);
  StatesPathFinderPtr_t shPtr(ptr);
  ptr->init(shPtr);
  return shPtr;
}

StatesPathFinderPtr_t StatesPathFinder::copy() const {
  StatesPathFinder* ptr = new StatesPathFinder(*this);
  StatesPathFinderPtr_t shPtr(ptr);
  ptr->init(shPtr);
  return shPtr;
}

struct StatesPathFinder::GraphSearchData {
  StatePtr_t s1;
  // list of potential goal states
  // can be multiple if goal is defined as a set of constraints
  graph::States_t s2;

  // index of the transition list
  size_t idxSol;

  // Datas for findNextTransitions
  struct state_with_depth {
    StatePtr_t s;
    EdgePtr_t e;
    std::size_t l;  // depth to root
    std::size_t i;  // index in parent state_with_depths
    // constructor used for root node
    inline state_with_depth() : s(), e(), l(0), i(0) {}
    // constructor used for non-root node
    inline state_with_depth(EdgePtr_t _e, std::size_t _l, std::size_t _i)
        : s(_e->stateFrom()), e(_e), l(_l), i(_i) {}
  };
  typedef std::vector<state_with_depth> state_with_depths;
  typedef std::map<StatePtr_t, state_with_depths> StateMap_t;
  /// std::size_t is the index in state_with_depths at StateMap_t::iterator
  struct state_with_depth_ptr_t {
    StateMap_t::iterator state;
    std::size_t parentIdx;
    state_with_depth_ptr_t(const StateMap_t::iterator& it, std::size_t idx)
        : state(it), parentIdx(idx) {}
  };
  typedef std::deque<state_with_depth_ptr_t> Deque_t;
  // vector of pointers to state with depth
  typedef std::vector<state_with_depth_ptr_t> state_with_depths_t;
  // transition lists exceeding this depth in the constraint graph will not be
  // considered
  std::size_t maxDepth;
  // map each state X to a list of preceding states in transition lists that
  // visit X state_with_depth struct gives info to trace the entire transition
  // lists
  StateMap_t parent1;
  // store a vector of pointers to the end state of each transition list
  state_with_depths_t solutions;
  // the frontier of the graph search
  // consists states that have not been expanded on
  Deque_t queue1;

  // track index of first transition list that has not been checked out
  // only needed when goal is defined as set of constraints
  size_t queueIt;

  const state_with_depth& getParent(const state_with_depth_ptr_t& _p) const {
    const state_with_depths& parents = _p.state->second;
    return parents[_p.parentIdx];
  }

  // add initial state (root node) to the map parent1
  state_with_depth_ptr_t addInitState() {
    StateMap_t::iterator next =
        parent1.insert(StateMap_t::value_type(s1, state_with_depths(1))).first;
    return state_with_depth_ptr_t(next, 0);
  }

  // store a transition to the map parent1
  state_with_depth_ptr_t addParent(const state_with_depth_ptr_t& _p,
                                   const EdgePtr_t& transition) {
    const state_with_depths& parents = _p.state->second;
    const state_with_depth& from = parents[_p.parentIdx];

    // Insert state to if necessary
    StateMap_t::iterator next =
        parent1
            .insert(StateMap_t::value_type(transition->stateTo(),
                                           state_with_depths()))
            .first;

    next->second.push_back(
        state_with_depth(transition, from.l + 1, _p.parentIdx));

    return state_with_depth_ptr_t(next, next->second.size() - 1);
  }
};

static bool containsLevelSet(const graph::EdgePtr_t& e) {
  graph::WaypointEdgePtr_t we = HPP_DYNAMIC_PTR_CAST(graph::WaypointEdge, e);
  if (!we) return false;
  for (std::size_t i = 0; i <= we->nbWaypoints(); i++) {
    graph::LevelSetEdgePtr_t lse =
        HPP_DYNAMIC_PTR_CAST(graph::LevelSetEdge, we->waypoint(i));
    if (lse) return true;
  }
  return false;
}

static bool isLoopTransition(const graph::EdgePtr_t& transition) {
  return transition->stateTo() == transition->stateFrom();
}

void StatesPathFinder::gatherGraphConstraints() {
  typedef graph::Edge Edge;
  typedef graph::EdgePtr_t EdgePtr_t;
  typedef graph::GraphPtr_t GraphPtr_t;
  typedef constraints::solver::BySubstitution Solver_t;

  GraphPtr_t cg(problem_->constraintGraph());
  const ConstraintsAndComplements_t& cac(cg->constraintsAndComplements());
  for (std::size_t i = 0; i < cg->nbComponents(); ++i) {
    EdgePtr_t edge(HPP_DYNAMIC_PTR_CAST(Edge, cg->get(i).lock()));

    // only gather the edge constraints
    if (!edge) continue;

    // Don't even consider level set edges
    if (containsLevelSet(edge)) continue;

    const Solver_t& solver(edge->pathConstraint()->configProjector()->solver());
    const NumericalConstraints_t& constraints(solver.numericalConstraints());
    for (NumericalConstraints_t::const_iterator it(constraints.begin());
         it != constraints.end(); ++it) {
      // only look at parameterized constraint
      if ((*it)->parameterSize() == 0) continue;

      const std::string& name((*it)->function().name());
      // if constraint is in map, no need to add it
      if (index_.find(name) != index_.end()) continue;

      index_[name] = constraints_.size();
      // Check whether constraint is equivalent to a previous one
      for (NumericalConstraints_t::const_iterator it1(constraints_.begin());
           it1 != constraints_.end(); ++it1) {
        for (ConstraintsAndComplements_t::const_iterator it2(cac.begin());
             it2 != cac.end(); ++it2) {
          if (((**it1 == *(it2->complement)) && (**it == *(it2->both))) ||
              ((**it1 == *(it2->both)) && (**it == *(it2->complement)))) {
            assert(sameRightHandSide_.count(*it1) == 0);
            assert(sameRightHandSide_.count(*it) == 0);
            sameRightHandSide_[*it1] = *it;
            sameRightHandSide_[*it] = *it1;
          }
        }
      }
      constraints_.push_back(*it);
      hppDout(info, "Adding constraint \"" << name << "\"");
      hppDout(info, "Edge \"" << edge->name() << "\"");
      hppDout(info, "parameter size: " << (*it)->parameterSize());
    }
  }
  // constraint/both is the intersection of both constraint and
  // constraint/complement
  for (ConstraintsAndComplements_t::const_iterator it(cac.begin());
       it != cac.end(); ++it) {
    stricterConstraints_[it->constraint] = it->both;
    stricterConstraints_[it->complement] = it->both;
  }
}

bool StatesPathFinder::findTransitions(GraphSearchData& d) const {
  // all potential solutions should be attempted before finding more
  if (d.idxSol < d.solutions.size()) return false;
  bool done = false;
  while (!d.queue1.empty() && !done) {
    GraphSearchData::state_with_depth_ptr_t _state = d.queue1.front();

    const GraphSearchData::state_with_depth& parent = d.getParent(_state);
    if (parent.l >= d.maxDepth) return true;
    d.queue1.pop_front();

    const Neighbors_t& neighbors = _state.state->first->neighbors();
    for (Neighbors_t::const_iterator _n = neighbors.begin();
         _n != neighbors.end(); ++_n) {
      EdgePtr_t transition = _n->second;

      // Don't even consider level set edges
      if (containsLevelSet(transition)) continue;

      // Avoid identical consecutive transition
      if (transition == parent.e) continue;

      // Avoid loop transitions
      if (isLoopTransition(transition)) continue;

      // Insert the end state of the new path to parent
      GraphSearchData::state_with_depth_ptr_t endState =
          d.addParent(_state, transition);
      d.queue1.push_back(endState);

      // done if last state is one of potential goal states
      if (std::find(d.s2.begin(), d.s2.end(), transition->stateTo()) !=
          d.s2.end()) {
        done = true;
        d.solutions.push_back(endState);
      }
    }
  }
  // return true if search is exhausted and goal state not found
  if (!done) return true;
  return false;
}

Edges_t StatesPathFinder::getTransitionList(const GraphSearchData& d,
                                            const std::size_t& idxSol) const {
  Edges_t transitions;
  if (idxSol >= d.solutions.size()) return transitions;

  const GraphSearchData::state_with_depth_ptr_t endState = d.solutions[idxSol];
  const GraphSearchData::state_with_depth* current = &d.getParent(endState);
  transitions.reserve(current->l);
  graph::WaypointEdgePtr_t we;
  while (current->e) {
    assert(current->l > 0);
    we = HPP_DYNAMIC_PTR_CAST(graph::WaypointEdge, current->e);
    if (we) {
      for (int i = (int)we->nbWaypoints(); i >= 0; --i)
        transitions.push_back(we->waypoint(i));
    } else {
      transitions.push_back(current->e);
    }
    current = &d.parent1.at(current->s)[current->i];
  }
  std::reverse(transitions.begin(), transitions.end());
  return transitions;
}

struct StatesPathFinder::OptimizationData {
  typedef constraints::solver::HierarchicalIterative::Saturation_t Saturation_t;
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
  };  // enum RightHandSideStatus_t
  const std::size_t N, nq, nv;
  std::vector<Solver_t> solvers;
  std::vector<bool> isTargetWaypoint;
  // Waypoints lying in each intermediate state
  matrix_t waypoint;
  const Configuration_t q1;
  Configuration_t q2;
  core::DevicePtr_t robot;
  // Matrix specifying for each constraint and each waypoint how
  // the right hand side is initialized in the solver.
  Eigen::Matrix<LiegroupElement, Eigen::Dynamic, Eigen::Dynamic> M_rhs;
  Eigen::Matrix<RightHandSideStatus_t, Eigen::Dynamic, Eigen::Dynamic> M_status;
  // Number of trials to generate each waypoint configuration
  // _solveGoalConfig: whether we need to solve for goal configuration
  OptimizationData(const core::ProblemConstPtr_t _problem,
                   ConfigurationIn_t _q1, ConfigurationIn_t _q2,
                   const Edges_t& transitions, const bool _solveGoalConfig)
      : N(_solveGoalConfig ? transitions.size() : transitions.size() - 1),
        nq(_problem->robot()->configSize()),
        nv(_problem->robot()->numberDof()),
        solvers(N, _problem->robot()->configSpace()),
        waypoint(nq, N),
        q1(_q1),
        robot(_problem->robot()),
        M_rhs(),
        M_status() {
    if (!_solveGoalConfig) {
      assert(_q2.size() > 0);
      q2 = _q2;
    }
    waypoint.setZero();
    for (auto solver : solvers) {
      // Set maximal number of iterations for each solver
      solver.maxIterations(
          _problem->getParameter("StatesPathFinder/maxIteration").intValue());
      // Set error threshold for each solver
      solver.errorThreshold(
          _problem->getParameter("StatesPathFinder/errorThreshold")
              .floatValue());
    }
    assert(transitions.size() > 0);
    isTargetWaypoint.assign(transitions.size(), false);
    for (std::size_t i = 0; i < transitions.size(); i++)
      isTargetWaypoint[i] = transitions[i]->stateTo()->isWaypoint();
  }
};

bool StatesPathFinder::checkConstantRightHandSide(size_type index) {
  // a goal configuration is required to check if constraint is satisfied
  // between initial and final configurations
  assert(!goalDefinedByConstraints_);
  OptimizationData& d = *optData_;
  const ImplicitPtr_t c(constraints_[index]);
  LiegroupElement rhsInit(c->function().outputSpace());
  c->rightHandSideFromConfig(d.q1, rhsInit);
  LiegroupElement rhsGoal(c->function().outputSpace());
  c->rightHandSideFromConfig(d.q2, rhsGoal);
  // Check that right hand sides are close to each other
  value_type eps(problem_->constraintGraph()->errorThreshold());
  value_type eps2(eps * eps);
  if ((rhsGoal - rhsInit).squaredNorm() > eps2) {
    return false;
  }
  // Matrix of solver right hand sides
  for (size_type j = 0; j < d.M_rhs.cols(); ++j) {
    d.M_rhs(index, j) = rhsInit;
  }
  return true;
}

bool StatesPathFinder::checkWaypointRightHandSide(std::size_t ictr,
                                                  std::size_t jslv) const {
  const OptimizationData& d = *optData_;
  ImplicitPtr_t c = constraints_[ictr]->copy();
  LiegroupElement rhsNow(c->function().outputSpace());
  assert(rhsNow.size() == c->rightHandSideSize());
  c->rightHandSideFromConfig(d.waypoint.col(jslv), rhsNow);
  c = constraints_[ictr]->copy();
  LiegroupElement rhsOther(c->function().outputSpace());
  switch (d.M_status(ictr, jslv)) {
    case OptimizationData::EQUAL_TO_INIT:
      c->rightHandSideFromConfig(d.q1, rhsOther);
      break;
    case OptimizationData::EQUAL_TO_GOAL:
      assert(!goalDefinedByConstraints_);
      c->rightHandSideFromConfig(d.q2, rhsOther);
      break;
    case OptimizationData::EQUAL_TO_PREVIOUS:
      c->rightHandSideFromConfig(d.waypoint.col(jslv - 1), rhsOther);
      break;
    case OptimizationData::ABSENT:
    default:
      return true;
  }
  hpp::pinocchio::vector_t diff = rhsOther - rhsNow;
  hpp::pinocchio::vector_t diffmask =
      hpp::pinocchio::vector_t::Zero(diff.size());
  for (auto k : c->activeRows())  // filter with constraint mask
    for (size_type kk = k.first; kk < k.first + k.second; kk++)
      diffmask[kk] = diff[kk];
  value_type eps(problem_->constraintGraph()->errorThreshold());
  value_type eps2(eps * eps);
  return diffmask.squaredNorm() < eps2;
}

bool StatesPathFinder::checkWaypointRightHandSide(std::size_t jslv) const {
  for (std::size_t ictr = 0; ictr < constraints_.size(); ictr++)
    if (!checkWaypointRightHandSide(ictr, jslv)) return false;
  return true;
}

void StatesPathFinder::displayRhsMatrix() {
  OptimizationData& d = *optData_;
  Eigen::Matrix<LiegroupElement, Eigen::Dynamic, Eigen::Dynamic>& m = d.M_rhs;

  for (std::size_t i = 0; i < constraints_.size(); i++) {
    const ImplicitPtr_t& constraint = constraints_[i];
    for (std::size_t j = 0; j < d.solvers.size(); j++) {
      const vectorIn_t& config = d.waypoint.col(j);
      LiegroupElement le(constraint->function().outputSpace());
      constraint->rightHandSideFromConfig(d.waypoint.col(j), le);
      m(i, j) = le;
    }
  }

  std::ostringstream oss;
  oss.precision(2);
  oss << "\\documentclass[12pt,landscape]{article}" << std::endl;
  oss << "\\usepackage[a3paper]{geometry}" << std::endl;
  oss << "\\begin {document}" << std::endl;

  for (size_type ii = 0; ii < (m.cols() + 7) / 8; ii++) {
    size_type j0 = ii * 8;
    size_type j1 = std::min(ii * 8 + 8, m.cols());
    size_type dj = j1 - j0;
    oss << "\\begin {tabular}{";
    for (size_type j = 0; j < dj + 2; ++j) oss << "c";
    oss << "}" << std::endl;
    oss << "Constraint & mask";
    for (size_type j = j0; j < j1; ++j) oss << " & WP" << j;
    oss << "\\\\" << std::endl;
    for (size_type i = 0; i < m.rows(); ++i) {
      std::vector<int> mask(constraints_[i]->parameterSize());
      for (auto k : constraints_[i]->activeRows())
        for (size_type kk = k.first; kk < k.first + k.second; kk++)
          mask[kk] = 1;
      std::ostringstream oss1;
      oss1.precision(2);
      std::ostringstream oss2;
      oss2.precision(2);
      oss1 << constraints_[i]->function().name() << " & ";

      oss1 << "$\\left(\\begin{array}{c} ";
      for (std::size_t k = 0; k < mask.size(); ++k) {
        oss1 << mask[k] << "\\\\ ";
      }
      oss1 << "\\end{array}\\right)$" << std::endl;
      oss1 << " & " << std::endl;

      for (size_type j = j0; j < j1; ++j) {
        if (d.M_status(i, j) != OptimizationData::ABSENT ||
            (j < m.cols() - 1 &&
             d.M_status(i, j + 1) == OptimizationData::EQUAL_TO_PREVIOUS)) {
          oss2 << "$\\left(\\begin{array}{c} ";
          for (size_type k = 0; k < m(i, j).size(); ++k) {
            oss2 << ((abs(m(i, j).vector()[k]) < 1e-6) ? 0
                                                       : m(i, j).vector()[k])
                 << "\\\\ ";
          }
          oss2 << "\\end{array}\\right)$" << std::endl;
        }
        if (j < j1 - 1) {
          oss2 << " & " << std::endl;
        }
      }
      std::string str2 = oss2.str();
      if (str2.size() > 50) {  // don't display constraints used nowhere
        oss << oss1.str() << str2 << "\\\\" << std::endl;
      }
    }
    oss << "\\end{tabular}" << std::endl << std::endl;
  }
  oss << "\\end{document}" << std::endl;

  std::string s = oss.str();
  std::string s2 = "";
  for (std::size_t i = 0; i < s.size(); i++) {
    if (s[i] == '_')
      s2 += "\\_";
    else
      s2.push_back(s[i]);
  }
  hppDout(info, s2);
}

void StatesPathFinder::displayStatusMatrix(const graph::Edges_t& transitions) {
  const Eigen::Matrix<OptimizationData::RightHandSideStatus_t, Eigen::Dynamic,
                      Eigen::Dynamic>& m = optData_->M_status;
  std::ostringstream oss;
  oss.precision(5);
  oss << "\\documentclass[12pt,landscape]{article}" << std::endl;
  oss << "\\usepackage[a3paper]{geometry}" << std::endl;
  oss << "\\begin {document}" << std::endl;
  oss << "\\paragraph{Edges}" << std::endl;
  oss << "\\begin{enumerate}" << std::endl;
  for (auto edge : transitions) {
    oss << "\\item \\texttt{" << edge->name() << "}" << std::endl;
  }
  oss << "\\end{enumerate}" << std::endl;
  oss << "\\begin {tabular}{l|";
  for (size_type j = 0; j < m.cols(); ++j)
    if (transitions[j]->stateTo()->isWaypoint())
      oss << "c";
    else
      oss << "|c|";
  oss << "|}" << std::endl;
  oss << "Constraint";
  for (size_type j = 0; j < m.cols(); ++j) oss << " & " << j + 1;
  oss << "\\\\" << std::endl;
  for (size_type i = 0; i < m.rows(); ++i) {
    oss << "\\texttt{" << constraints_[i]->function().name() << "} & "
        << std::endl;
    for (size_type j = 0; j < m.cols(); ++j) {
      oss << m(i, j);
      if (j < m.cols() - 1) oss << " & ";
    }
    oss << "\\\\" << std::endl;
  }
  oss << "\\end{tabular}" << std::endl;
  oss << "\\end{document}" << std::endl;

  std::string s = oss.str();
  std::string s2 = "";
  for (std::size_t i = 0; i < s.size(); i++) {
    if (s[i] == '_')
      s2 += "\\_";
    else
      s2.push_back(s[i]);
  }
  hppDout(info, s2);
}

bool StatesPathFinder::contains(const Solver_t& solver,
                                const ImplicitPtr_t& c) const {
  if (solver.contains(c)) return true;
  std::map<ImplicitPtr_t, ImplicitPtr_t>::const_iterator it(
      sameRightHandSide_.find(c));
  if (it != sameRightHandSide_.end() && solver.contains(it->second))
    return true;
  return false;
}

bool StatesPathFinder::containsStricter(const Solver_t& solver,
                                        const ImplicitPtr_t& c) const {
  if (solver.contains(c)) return true;
  std::map<ImplicitPtr_t, ImplicitPtr_t>::const_iterator it(
      stricterConstraints_.find(c));
  if (it != stricterConstraints_.end() && solver.contains(it->second))
    return true;
  return false;
}

bool StatesPathFinder::buildOptimizationProblem(
    const graph::Edges_t& transitions) {
  OptimizationData& d = *optData_;
  // if no waypoint, check init and goal has same RHS
  if (d.N == 0) {
    assert(transitions.size() == 1);
    assert(!goalDefinedByConstraints_);
    size_type index = 0;
    for (NumericalConstraints_t::const_iterator it(constraints_.begin());
         it != constraints_.end(); ++it, ++index) {
      const ImplicitPtr_t& c(*it);
      // Get transition solver
      const Solver_t& trSolver(
          transitions[0]->pathConstraint()->configProjector()->solver());
      if (!contains(trSolver, c)) continue;
      if (!checkConstantRightHandSide(index)) return false;
    }
    return true;
  }
  d.M_status.resize(constraints_.size(), d.N);
  d.M_status.fill(OptimizationData::ABSENT);
  d.M_rhs.resize(constraints_.size(), d.N);
  d.M_rhs.fill(LiegroupElement());
  size_type index = 0;
  // Loop over constraints
  for (NumericalConstraints_t::const_iterator it(constraints_.begin());
       it != constraints_.end(); ++it, ++index) {
    const ImplicitPtr_t& c(*it);
    // Loop forward over waypoints to determine right hand sides equal
    // to initial configuration or previous configuration
    for (std::size_t j = 0; j < d.N; ++j) {
      // Get transition solver
      const Solver_t& trSolver(
          transitions[j]->pathConstraint()->configProjector()->solver());
      if (!contains(trSolver, c)) continue;

      if ((j == 0) ||
          d.M_status(index, j - 1) == OptimizationData::EQUAL_TO_INIT) {
        d.M_status(index, j) = OptimizationData::EQUAL_TO_INIT;
      } else {
        d.M_status(index, j) = OptimizationData::EQUAL_TO_PREVIOUS;
      }
    }
    // If the goal configuration is not given
    // no need to determine if RHS equal to goal configuration
    if (goalDefinedByConstraints_) {
      continue;
    }
    // Loop backward over waypoints to determine right hand sides equal
    // to final configuration
    for (size_type j = d.N - 1; j > 0; --j) {
      // Get transition solver
      const Solver_t& trSolver(transitions[(std::size_t)j + 1]
                                   ->pathConstraint()
                                   ->configProjector()
                                   ->solver());
      if (!contains(trSolver, c)) break;

      if ((j == (size_type)d.N - 1) ||
          d.M_status(index, j + 1) == OptimizationData::EQUAL_TO_GOAL) {
        // if constraint right hand side is already equal to
        // initial config, check that right hand side is equal
        // for init and goal configs.
        if (d.M_status(index, j) == OptimizationData::EQUAL_TO_INIT) {
          if (checkConstantRightHandSide(index)) {
            // stop for this constraint
            break;
          } else {
            // Right hand side of constraint should be equal along the
            // whole path but is different at init and at goal configs.
            return false;
          }
        } else {
          d.M_status(index, j) = OptimizationData::EQUAL_TO_GOAL;
        }
      }
    }
  }  // for (NumericalConstraints_t::const_iterator it
#ifdef HPP_DEBUG
  displayStatusMatrix(transitions);
#endif
  // Fill solvers with target constraints of transition
  for (std::size_t j = 0; j < d.N; ++j) {
    d.solvers[j] =
        transitions[j]->targetConstraint()->configProjector()->solver();
    if (goalDefinedByConstraints_) {
      continue;
    }
    // when goal configuration is given, and if something
    // (eg relative pose of two objects grasping) is fixed until goal,
    // we need to propagate the constraint to an earlier solver,
    // otherwise the chance it solves for the correct config is very low
    if (j > 0 && j < d.N - 1) {
      const Solver_t& otherSolver =
          transitions[j + 1]->pathConstraint()->configProjector()->solver();
      for (std::size_t i = 0; i < constraints_.size(); i++) {
        // transition from j-1 to j does not contain this constraint
        // transition from j to j+1 (all the way to goal) has constraint
        // constraint must be added to solve for waypoint at j (WP_j+1)
        if (d.M_status(i, j - 1) == OptimizationData::ABSENT &&
            d.M_status(i, j) == OptimizationData::EQUAL_TO_GOAL &&
            !contains(d.solvers[j], constraints_[i]) &&
            otherSolver.contains(constraints_[i])) {
          d.solvers[j].add(constraints_[i]);
          hppDout(info, "Adding missing constraint "
                            << constraints_[i]->function().name()
                            << " to solver for waypoint" << j + 1);
        }
      }
    }
  }

  // if goal is defined by constraints, some goal constraints may be
  // missing from the end state. we should add these constraints to solver
  // for the config in the final state
  if (goalDefinedByConstraints_) {
    for (auto goalConstraint : goalConstraints_) {
      if (!containsStricter(d.solvers[d.N - 1], goalConstraint)) {
        d.solvers[d.N - 1].add(goalConstraint);
        hppDout(info, "Adding goal constraint "
                          << goalConstraint->function().name()
                          << " to solver for waypoint" << d.N);
      }
    }
  }

  return true;
}

bool StatesPathFinder::checkSolverRightHandSide(std::size_t ictr,
                                                std::size_t jslv) const {
  const OptimizationData& d = *optData_;
  ImplicitPtr_t c = constraints_[ictr]->copy();
  const Solver_t& solver = d.solvers[jslv];
  vector_t rhs(c->rightHandSideSize());
  solver.getRightHandSide(c, rhs);
  LiegroupElement rhsNow(c->function().outputSpace());
  assert(rhsNow.size() == rhs.size());
  rhsNow.vector() = rhs;
  LiegroupElement rhsOther(c->function().outputSpace());
  switch (d.M_status(ictr, jslv)) {
    case OptimizationData::EQUAL_TO_INIT:
      c->rightHandSideFromConfig(d.q1, rhsOther);
      break;
    case OptimizationData::EQUAL_TO_GOAL:
      assert(!goalDefinedByConstraints_);
      c->rightHandSideFromConfig(d.q2, rhsOther);
      break;
    case OptimizationData::EQUAL_TO_PREVIOUS:
      c->rightHandSideFromConfig(d.waypoint.col(jslv - 1), rhsOther);
      break;
    case OptimizationData::ABSENT:
    default:
      return true;
  }
  hpp::pinocchio::vector_t diff = rhsOther - rhsNow;
  hpp::pinocchio::vector_t diffmask =
      hpp::pinocchio::vector_t::Zero(diff.size());
  for (auto k : c->activeRows())  // filter with constraint mask
    for (size_type kk = k.first; kk < k.first + k.second; kk++)
      diffmask[kk] = diff[kk];
  value_type eps(problem_->constraintGraph()->errorThreshold());
  value_type eps2(eps * eps);
  if (diffmask.squaredNorm() > eps2)
    hppDout(warning, diffmask.squaredNorm() << " vs " << eps2);
  return diffmask.squaredNorm() < eps2;
}

bool StatesPathFinder::checkSolverRightHandSide(std::size_t jslv) const {
  for (std::size_t ictr = 0; ictr < constraints_.size(); ictr++)
    if (!checkSolverRightHandSide(ictr, jslv)) return false;
  return true;
}

bool StatesPathFinder::buildOptimizationProblemFromNames(
    std::vector<std::string> names) {
  graph::Edges_t transitions;
  graph::GraphPtr_t cg(problem_->constraintGraph());
  for (const std::string& name : names) {
    for (std::size_t i = 0; i < cg->nbComponents(); ++i) {
      graph::EdgePtr_t edge(
          HPP_DYNAMIC_PTR_CAST(graph::Edge, cg->get(i).lock()));
      if (edge && edge->name() == name) transitions.push_back(edge);
    }
  }
  return buildOptimizationProblem(transitions);
}

void StatesPathFinder::preInitializeRHS(std::size_t j, Configuration_t& q) {
  OptimizationData& d = *optData_;
  Solver_t& solver(d.solvers[j]);
  for (std::size_t i = 0; i < constraints_.size(); ++i) {
    const ImplicitPtr_t& c(constraints_[i]);
    bool ok = true;
    switch (d.M_status((size_type)i, (size_type)j)) {
      case OptimizationData::EQUAL_TO_INIT:
        ok = solver.rightHandSideFromConfig(c, d.q1);
        break;
      case OptimizationData::EQUAL_TO_GOAL:
        assert(!goalDefinedByConstraints_);
        ok = solver.rightHandSideFromConfig(c, d.q2);
        break;
      case OptimizationData::EQUAL_TO_PREVIOUS:
        ok = solver.rightHandSideFromConfig(c, q);
        break;
      case OptimizationData::ABSENT:
      default:;
    }
    ok |= contains(solver, constraints_[i]);
    if (!ok) {
      std::ostringstream err_msg;
      err_msg << "\nConstraint " << i << " missing for waypoint " << j + 1
              << " (" << c->function().name() << ")\n"
              << "The constraints in this solver are:\n";
      for (const std::string& name : constraintNamesFromSolverAtWaypoint(j + 1))
        err_msg << name << "\n";
      hppDout(warning, err_msg.str());
    }
    assert(ok);
  }
}

bool StatesPathFinder::analyseOptimizationProblem(
    const graph::Edges_t& transitions, core::ProblemConstPtr_t _problem) {
  typedef constraints::JointConstPtr_t JointConstPtr_t;
  typedef core::RelativeMotion RelativeMotion;

  OptimizationData& d = *optData_;
  // map from pair of joint indices to vectors of constraints
  typedef std::map<std::pair<size_type, size_type>, NumericalConstraints_t>
      JointConstraintMap;
  JointConstraintMap jcmap;

  // iterate over all the transitions, propagate only constrained pairs
  for (std::size_t i = 0; i <= transitions.size() - 1; ++i) {
    // get index of the transition
    std::size_t transIdx = transitions.size() - 1 - i;
    const EdgePtr_t& edge = transitions[transIdx];
    RelativeMotion::matrix_type m = edge->relativeMotion();

    // check through the pairs already existing in jcmap
    JointConstraintMap::iterator it = jcmap.begin();
    while (it != jcmap.end()) {
      RelativeMotion::RelativeMotionType rmt =
          m(it->first.first, it->first.second);
      if (rmt == RelativeMotion::RelativeMotionType::Unconstrained) {
        JointConstraintMap::iterator toErase = it;
        ++it;
        jcmap.erase(toErase);
      } else {
        ++it;
      }
    }
    NumericalConstraints_t currentConstraints;
    if (transIdx == transitions.size() - 1 && !goalDefinedByConstraints_) {
      // get the constraints from the goal state
      currentConstraints = transitions[transIdx]
                               ->targetConstraint()
                               ->configProjector()
                               ->solver()
                               .constraints();
    } else {
      currentConstraints = d.solvers[transIdx].constraints();
    }
    // loop through all constraints in the target node of the transition
    for (auto constraint : currentConstraints) {
      std::pair<JointConstPtr_t, JointConstPtr_t> jointPair =
          constraint->functionPtr()->dependsOnRelPoseBetween(_problem->robot());
      JointConstPtr_t joint1 = jointPair.first;
      size_type index1 = RelativeMotion::idx(joint1);
      JointConstPtr_t joint2 = jointPair.second;
      size_type index2 = RelativeMotion::idx(joint2);

      // ignore constraint if it involves the same joint
      if (index1 == index2) continue;

      // check that the two joints are constrained in the transition
      RelativeMotion::RelativeMotionType rmt = m(index1, index2);
      if (rmt == RelativeMotion::RelativeMotionType::Unconstrained) continue;

      // insert if necessary
      JointConstraintMap::iterator next =
          jcmap
              .insert(JointConstraintMap::value_type(
                  std::make_pair(index1, index2), NumericalConstraints_t()))
              .first;
      // if constraint is not in map, insert it
      if (find_if(next->second.begin(), next->second.end(),
                  [&constraint](const ImplicitPtr_t& arg) {
                    return *arg == *constraint;
                  }) == next->second.end()) {
        next->second.push_back(constraint);
      }
    }
  }
  // at this point jcmap contains all the constraints that
  // - depend on relative pose between 2 joints j1 and j2,
  // - are present in the solver of any waypoint wp_i
  // - j1 and j2 are constrained by all the transitions from q_init to wp_i
  //
  // in the following we test that q_init satisfies the above constraints
  // otherwise the list of transitions is discarded
  if (jcmap.size() == 0) {
    return true;
  }

  Solver_t analyseSolver(_problem->robot()->configSpace());
  analyseSolver.errorThreshold(
      _problem->getParameter("StatesPathFinder/errorThreshold").floatValue());
  // iterate through all the pairs that are left,
  // and check that the initial config satisfies all the constraints
  for (JointConstraintMap::iterator it(jcmap.begin()); it != jcmap.end();
       it++) {
    NumericalConstraints_t constraintList = it->second;
    hppDout(info, "Constraints involving joints "
                      << it->first.first << " and " << it->first.second
                      << " should be satisfied at q init");
    for (NumericalConstraints_t::iterator ctrIt(constraintList.begin());
         ctrIt != constraintList.end(); ++ctrIt) {
      analyseSolver.add((*ctrIt)->copy());
    }
  }
  // initialize the right hand side with the initial config
  analyseSolver.rightHandSideFromConfig(q1_);
  if (analyseSolver.isSatisfied(q1_)) {
    return true;
  }
  hppDout(info,
          "Analysis found initial configuration does not satisfy constraint");
  return false;
}

void StatesPathFinder::initializeRHS(std::size_t j) {
  OptimizationData& d = *optData_;
  Solver_t& solver(d.solvers[j]);
  for (std::size_t i = 0; i < constraints_.size(); ++i) {
    const ImplicitPtr_t& c(constraints_[i]);
    bool ok = true;
    switch (d.M_status((size_type)i, (size_type)j)) {
      case OptimizationData::EQUAL_TO_PREVIOUS:
        assert(j != 0);
        ok = solver.rightHandSideFromConfig(c, d.waypoint.col(j - 1));
        break;
      case OptimizationData::EQUAL_TO_INIT:
        ok = solver.rightHandSideFromConfig(c, d.q1);
        break;
      case OptimizationData::EQUAL_TO_GOAL:
        assert(!goalDefinedByConstraints_);
        ok = solver.rightHandSideFromConfig(c, d.q2);
        break;
      case OptimizationData::ABSENT:
      default:;
    }
    ok |= contains(solver, constraints_[i]);
    if (!ok) {
      std::ostringstream err_msg;
      err_msg << "\nConstraint " << i << " missing for waypoint " << j + 1
              << " (" << c->function().name() << ")\n"
              << "The constraints in this solver are:\n";
      for (const std::string& name : constraintNamesFromSolverAtWaypoint(j + 1))
        err_msg << name << "\n";
      hppDout(warning, err_msg.str());
    }
    assert(ok);
  }
}

void StatesPathFinder::initWPRandom(std::size_t wp) {
  assert(wp >= 1 && wp <= (std::size_t)optData_->waypoint.cols());
  initializeRHS(wp - 1);
  optData_->waypoint.col(wp - 1) = problem()->configurationShooter()->shoot();
}
void StatesPathFinder::initWPNear(std::size_t wp) {
  assert(wp >= 1 && wp <= (std::size_t)optData_->waypoint.cols());
  initializeRHS(wp - 1);
  if (wp == 1)
    optData_->waypoint.col(wp - 1) = optData_->q1;
  else
    optData_->waypoint.col(wp - 1) = optData_->waypoint.col(wp - 2);
}
void StatesPathFinder::initWP(std::size_t wp, ConfigurationIn_t q) {
  assert(wp >= 1 && wp <= (std::size_t)optData_->waypoint.cols());
  initializeRHS(wp - 1);
  optData_->waypoint.col(wp - 1) = q;
}

StatesPathFinder::SolveStepStatus StatesPathFinder::solveStep(std::size_t wp) {
  assert(wp >= 1 && wp <= (std::size_t)optData_->waypoint.cols());
  std::size_t j = wp - 1;
  Solver_t& solver(optData_->solvers[j]);
  Solver_t::Status status =
      solver.solve(optData_->waypoint.col(j),
                   constraints::solver::lineSearch::Backtracking());
  if (status == Solver_t::SUCCESS) {
    assert(checkWaypointRightHandSide(j));
    const graph::Edges_t& edges = lastBuiltTransitions_;
    core::ValidationReportPtr_t report;
    // check collision based on preceeding edge to the waypoint
    if (!edges[j]->pathValidation()->validate(optData_->waypoint.col(j),
                                              report))
      return SolveStepStatus::COLLISION_BEFORE;
    // if wp is not the goal node, check collision based on following edge
    if (j < edges.size() - 1 && !edges[j + 1]->pathValidation()->validate(
                                    optData_->waypoint.col(j), report)) {
      return SolveStepStatus::COLLISION_AFTER;
    }
    return SolveStepStatus::VALID_SOLUTION;
  }
  return SolveStepStatus::NO_SOLUTION;
}

std::string StatesPathFinder::displayConfigsSolved() const {
  const OptimizationData& d = *optData_;
  std::ostringstream oss;
  oss << "configs = [" << std::endl;
  oss << "  " << pinocchio::displayConfig(d.q1) << ",  # 0" << std::endl;
  for (size_type j = 0; j < d.waypoint.cols(); j++)
    oss << "  " << pinocchio::displayConfig(d.waypoint.col(j)) << ",  # "
        << j + 1 << std::endl;
  if (!goalDefinedByConstraints_) {
    oss << "  " << pinocchio::displayConfig(d.q2) << "  # "
        << d.waypoint.cols() + 1 << std::endl;
  }
  oss << "]" << std::endl;
  std::string ans = oss.str();
  hppDout(info, ans);
  return ans;
}

// Get a configuration in accordance with the statuts matrix at a step j for the
// constraint i
Configuration_t StatesPathFinder::getConfigStatus(size_type i,
                                                  size_type j) const {
  switch (optData_->M_status(i, j)) {
    case OptimizationData::EQUAL_TO_PREVIOUS:
      return optData_->waypoint.col(j - 1);
    case OptimizationData::EQUAL_TO_INIT:
      return optData_->q1;
    case OptimizationData::EQUAL_TO_GOAL:
      return optData_->q2;
    default:
      return optData_->waypoint.col(j);
  }
}

// Get the right hand side of a constraint w.r.t a set configuration for this
// constraint
vector_t StatesPathFinder::getConstraintRHS(ImplicitPtr_t constraint,
                                            Configuration_t q) const {
  LiegroupElement rhs(constraint->copy()->function().outputSpace());
  constraint->rightHandSideFromConfig(q, rhs);
  return rhs.vector();
}

// Hash a vector of right hand side into a long unsigned integer
size_t StatesPathFinder::hashRHS(vector_t rhs) const {
  std::stringstream ss;
  ss << std::setprecision(3) << (0.01 < rhs.array().abs()).select(rhs, 0.0f);
  return std::hash<std::string>{}(ss.str());
}

// Check if a solution (a list of transitions) contains impossible to solve
// steps due to inevitable collisions. A step is impossible to solve if it has
// two constraints set from init or goal which have produced a collision between
// objects constrained by them.
// The list of such known constraint pairs are memorized in pairMap table and
// individually in constraintMap.
//
// Return : true if no impossible to solve steps, false otherwise
bool StatesPathFinder::checkSolvers(
    ConstraintMap_t const& pairMap,
    ConstraintMap_t const& constraintMap) const {
  // do nothing if there is no known incompatible constraint pairs.
  if (constraintMap.empty()) return true;

  // for each steps of the solution
  for (long unsigned i{0}; i < optData_->solvers.size(); i++) {
    std::vector<std::pair<ImplicitPtr_t, Configuration_t>> constraints{};

    // gather all constraints of this step which are set from init or goal
    // configuration
    for (long unsigned j{0}; j < constraints_.size(); j++) {
      auto c = constraints_[j];
      auto status = optData_->M_status(j, i);
      if (status != OptimizationData::EQUAL_TO_INIT &&
          status != OptimizationData::EQUAL_TO_GOAL)  // if not init or goal
        continue;
      auto q = getConfigStatus(j, i);
      auto name = std::hash<std::string>{}(c->function().name());
      if (constraintMap.count(name))
        constraints.push_back(std::make_pair(c, q));
    }

    // if there are less than two constraints gathered, go to next step
    if (constraints.size() < 2) continue;

    // else, check if there is a pair of constraints in the table of known
    // incompatible pairs
    for (auto& c1 : constraints) {
      auto rhs_1 = hashRHS(getConstraintRHS(std::get<0>(c1), std::get<1>(c1)));
      auto name_1 =
          std::hash<std::string>{}(std::get<0>(c1)->function().name());
      if (constraintMap.count(name_1))
        for (auto& c2 : constraints) {
          auto rhs_2 =
              hashRHS(getConstraintRHS(std::get<0>(c2), std::get<1>(c2)));
          auto name_2 =
              std::hash<std::string>{}(std::get<0>(c2)->function().name());
          auto namesPair = name_1 * name_2;
          auto rhsPair = rhs_1 * rhs_2;
          if (name_1 != name_2 && pairMap.count(namesPair) &&
              pairMap.at(namesPair) == rhsPair)
            return false;
        }
    }
  }

  return true;
}

core::JointConstPtr_t StatesPathFinder::maximalJoint(size_t const wp,
                                                     core::JointConstPtr_t a) {
  const auto& current_edge = lastBuiltTransitions_[wp + 1];
  core::RelativeMotion::matrix_type m = current_edge->relativeMotion();

  size_t ida = core::RelativeMotion::idx(a);

  core::JointConstPtr_t last = nullptr;
  core::JointConstPtr_t current = a;

  while (current != nullptr) {
    auto parent = current->parentJoint();
    size_t idp = core::RelativeMotion::idx(parent);
    last = current;
    if (parent && m(ida, idp))
      current = current->parentJoint();
    else
      break;
  }

  return last;
}

// For a certain step wp during solving check for collision impossible to solve.
// If there is any store the constraints involved and stop the resolution
//
// Return : true if there is no collision or impossible to solve ones, false
// otherwise.
bool StatesPathFinder::saveIncompatibleRHS(ConstraintMap_t& pairMap,
                                           ConstraintMap_t& constraintMap,
                                           size_type const wp) {
  core::ValidationReportPtr_t validationReport;
  auto q = optData_->waypoint.col(wp - 1);
  bool nocollision{true};

  core::CollisionValidationPtr_t collisionValidations =
      core::CollisionValidation::create(problem_->robot());
  collisionValidations->checkParameterized(true);
  collisionValidations->computeAllContacts(true);

  // If there was a collision in the last configuration
  if (!collisionValidations->validate(q, validationReport)) {
    auto allReports = HPP_DYNAMIC_PTR_CAST(core::AllCollisionsValidationReport,
                                           validationReport);

    // check all collision reports between joints
    for (auto& report : allReports->collisionReports) {
      // Store the two joints involved
      core::JointConstPtr_t j1 = report->object1->joint();
      core::JointConstPtr_t j2 = report->object2->joint();

      // check that there is indeed two joints
      if (!j1 || !j2) return nocollision;

      // get the maximal parent joint which are constrained with their child
      j1 = maximalJoint(wp, j1);
      j2 = maximalJoint(wp, j2);

      // Function to check if two joints are equals, bye their address is
      // nullptr, by their value otherwise
      auto equalJoints = [](core::JointConstPtr_t a, core::JointConstPtr_t b) {
        return (a && b) ? *a == *b : a == b;
      };

      // std::cout << "test are constrained for j1 & j2: " << areConstrained(wp,
      // j1, j2) << std::endl;

      typedef std::pair<core::JointConstPtr_t, size_t> jointOfConstraint;

      // Get all constraints which involve a joint
      auto associatedConstraints = [&](core::JointConstPtr_t j) {
        std::vector<jointOfConstraint> constraints{};
        for (size_t i{0}; i < constraints_.size(); i++) {
          auto constraint = constraints_[i];
          auto joints =
              constraint->doesConstrainRelPoseBetween(problem_->robot());
          if (equalJoints(j, std::get<0>(joints)) ||
              equalJoints(j, std::get<1>(joints))) {
            auto joint = (j == std::get<0>(joints)) ? std::get<1>(joints)
                                                    : std::get<0>(joints);
            constraints.push_back(std::make_pair(joint, i));
          }
        }
        return constraints;
      };

      // We get all the constraints which contain j1
      auto constraints_j1 = associatedConstraints(j1);

      const auto& current_edge = lastBuiltTransitions_[wp + 1];
      core::RelativeMotion::matrix_type m = current_edge->relativeMotion();

      std::vector<short> visited(constraints_.size(), 0);
      std::function<std::vector<int>(core::JointConstPtr_t, jointOfConstraint,
                                     int)>
          exploreJOC;

      // Check if a joint is indirectly co-constrained with an other, return the
      // indices of their respective constraints
      exploreJOC = [&](core::JointConstPtr_t j, jointOfConstraint current,
                       int initial) {
        int constraint_index = std::get<1>(current);
        visited[constraint_index]++;
        core::JointConstPtr_t current_joint = std::get<0>(current);
        auto iconstraint = constraints_[constraint_index];
        auto JOCs = associatedConstraints(current_joint);
        auto id_cj = core::RelativeMotion::idx(current_joint);

        for (auto& joc : JOCs) {
          int ci = std::get<1>(joc);
          core::JointConstPtr_t ji = std::get<0>(joc);
          auto id_ji = core::RelativeMotion::idx(ji);
          if (m(id_cj, id_ji) ==
              core::RelativeMotion::RelativeMotionType::Unconstrained)
            continue;
          if (equalJoints(ji, j))
            return std::vector<int>{initial, ci};
          else if (visited[ci] < 2) {
            return exploreJOC(j, joc, initial);
          }
        }
        return std::vector<int>{-1, -1};
      };

      // get the indices of the constraints associated to the two joints
      auto getIndices = [&] -> std::vector<int> {
        for (auto& joc : constraints_j1) {
          visited[std::get<1>(joc)] = 1;
          auto indices = exploreJOC(j2, joc, std::get<1>(joc));
          if (indices[0] >= 0 || indices[1] >= 0) return indices;
        }
        return std::vector<int>{-1, -1};
      };

      // indices contains the two indices of the constraints that constraint
      // pose of j1 with respect to j2, if any, or { -1, -1}
      auto indices = getIndices();

      // Make sure indices are all defined
      if (indices[0] < 0 || indices[1] < 0) return nocollision;

      // for each of the two constraint identified
      std::vector<std::pair<size_t, size_t>> constraints{};
      for (auto ic : indices) {
        // check that there are set from goal or init configurations
        auto status = optData_->M_status((size_t)ic, wp - 1);
        if (status != OptimizationData::EQUAL_TO_INIT &&
            status != OptimizationData::EQUAL_TO_GOAL)
          return nocollision;

        // if so prepare to store them in the tables of known incompatible
        // constraints
        auto c = constraints_[(size_t)ic];
        auto rhs = hashRHS(getConstraintRHS(c, q));
        auto name = std::hash<std::string>{}(c->function().name());

        constraints.push_back(std::make_pair(name, rhs));
      }

      // then add them both in the tables of incompatible constraints
      nocollision = false;

      // first, individually, in constraintMap
      for (auto& c : constraints)
        if (!constraintMap.count(std::get<0>(c)))
          constraintMap[std::get<0>(c)] = std::get<1>(c);

      // then, both merged together in pairMap
      auto names =
          std::get<0>(constraints[0]) * std::get<0>(constraints[1]);  // key
      auto rhs =
          std::get<1>(constraints[0]) * std::get<1>(constraints[1]);  // value

      if (!pairMap.count(names)) pairMap[names] = rhs;
    }
  }

  return nocollision;
}

bool StatesPathFinder::solveOptimizationProblem() {
  static ConstraintMap_t pairMap{};
  static ConstraintMap_t constraintMap{};

  // check if the solution is feasible (no inevitable collision), if not abort
  // the solving
  if (!checkSolvers(pairMap, constraintMap)) {
    hppDout(info, "Path is invalid for collision is inevitable");
    return false;
  }

  OptimizationData& d = *optData_;
  // Try to solve with sets of random configs for each waypoint
  std::size_t nTriesMax =
      problem_->getParameter("StatesPathFinder/nTriesUntilBacktrack")
          .intValue();
  std::size_t nTriesMax1 = nTriesMax * 10;  // more tries for the first waypoint
  std::size_t nFailsMax =
      nTriesMax * 20;  // fails before reseting the whole solution
  std::size_t nBadSolvesMax =
      nTriesMax * 50;  // bad solve fails before reseting the whole solution
  std::vector<std::size_t> nTriesDone(d.solvers.size() + 1, 0);
  std::size_t nFails = 0;
  std::size_t nBadSolves = 0;
  std::size_t wp = 1;      // waypoint index starting at 1 (wp 0 = q1)
  std::size_t wp_max = 0;  // all waypoints up to this index are valid solved
  matrix_t longestSolved(d.nq, d.N);
  longestSolved.setZero();
  while (wp <= d.solvers.size()) {
    if (wp == 1) {
      // stop if number of tries for the first waypoint exceeds
      if (nTriesDone[1] >= nTriesMax1) {
        // if cannot solve all the way, return longest VALID sequence
        d.waypoint = longestSolved;
        displayConfigsSolved();
        return false;
      }
      // Reset the fail counter while the solution is empty
      nFails = 0;
      nBadSolves = 0;

    } else if (nFails >= nFailsMax || nBadSolves >= nBadSolvesMax) {
      // Completely reset a solution when too many tries have failed

      // update the longest valid sequence of waypoints solved
      if (wp - 1 > wp_max) {
        // update the maximum index of valid waypoint
        wp_max = wp - 1;
        // save the sequence
        longestSolved.leftCols(wp_max) = d.waypoint.leftCols(wp_max);
      }

      std::fill(nTriesDone.begin() + 2, nTriesDone.end(), 0);
      wp = 1;

#ifdef HPP_DEBUG
      if (nFails >= nFailsMax) {
        hppDout(warning, " Solution "
                             << graphData_->idxSol
                             << ": too many collisions. Resetting back to WP1");
      }
      if (nBadSolves >= nBadSolvesMax) {
        hppDout(warning,
                " Solution "
                    << graphData_->idxSol
                    << ": too many bad solve statuses. Resetting back to WP1");
      }
#endif

      continue;

    } else if (nTriesDone[wp] >= nTriesMax) {
      // enough tries for a waypoint: backtrack

      // update the longest valid sequence of waypoints solved
      if (wp - 1 > wp_max) {
        // update the maximum index of valid waypoint
        wp_max = wp - 1;
        // save the sequence
        longestSolved.leftCols(wp_max) = d.waypoint.leftCols(wp_max);
      }

      do {
        nTriesDone[wp] = 0;
        wp--;  // backtrack: try a new solution for the latest waypoint
      } while (wp > 1 &&
               (d.isTargetWaypoint[wp - 1] || nTriesDone[wp] >= nTriesMax));

      continue;
    }

    // Initialize right hand sides, and
    // Choose a starting configuration for the solver.solve method:
    // - from previous waypoint if it's the first time we see this solver
    //   given current solvers 0 to j-1
    // - with a random configuration if the other initialization has been
    //   tried and failed
    if (nTriesDone[wp] == 0)
      initWPNear(wp);
    else
      initWPRandom(wp);

    nTriesDone[wp]++;  // Backtrack to last state when this gets too big

    SolveStepStatus out = solveStep(wp);
    hppDout(info, "solveStep exit code at WP" << wp << ": " << out);
    switch (out) {
      case SolveStepStatus::VALID_SOLUTION:  // Valid solution, go to next
                                             // waypoint
        wp++;
        break;
      case SolveStepStatus::NO_SOLUTION:  // Bad solve status, considered usual
                                          // so has higher threshold before
                                          // going back to first waypoint
        nBadSolves++;
        break;
      case SolveStepStatus::COLLISION_BEFORE:  // Collision. If that happens too
                                               // much, go back to first
                                               // waypoint
      case SolveStepStatus::COLLISION_AFTER:
        // if collision check that it is not due to non-solvable constraints,
        // if so, store the constraints involved and abort the solving
        if (!saveIncompatibleRHS(pairMap, constraintMap, wp)) {
          hppDout(info, "Path is invalid, found inevitable collision");
          return false;
        }
        nFails++;
        break;
      default:
        throw(std::logic_error("Unintended exit code for solveStep"));
    }
  }

  displayConfigsSolved();
  displayRhsMatrix();

  return true;
}

// Get list of configurations from solution of optimization problem
core::Configurations_t StatesPathFinder::getConfigList() const {
  OptimizationData& d = *optData_;
  core::Configurations_t pv;
  pv.push_back(d.q1);
  for (std::size_t i = 0; i < d.N; ++i) {
    pv.push_back(d.waypoint.col(i));
  }
  if (!goalDefinedByConstraints_) {
    pv.push_back(d.q2);
  }
  return pv;
}

// Loop over all the possible paths in the constraint graph between
// the state of the initial configuration
// and either [the state of the final configurations if given] OR
// [one of potential goal states if goal defined as set of constraints]
// and compute waypoint configurations in each state.
core::Configurations_t StatesPathFinder::computeConfigList(
    ConfigurationIn_t q1, ConfigurationIn_t q2) {
  const graph::GraphPtr_t& graph(problem_->constraintGraph());
  GraphSearchData& d = *graphData_;
  size_t& idxSol = d.idxSol;

  bool maxDepthReached;
  while (!(maxDepthReached = findTransitions(d))) {  // mut
    // if there is a working sequence, try it first before getting another
    // transition list
    Edges_t transitions = (nTryConfigList_ > 0)
                              ? lastBuiltTransitions_
                              : getTransitionList(d, idxSol);  // const, const
    while (!transitions.empty()) {
#ifdef HPP_DEBUG
      std::ostringstream ss;
      ss << " Trying solution " << idxSol << ": \n\t";
      for (std::size_t j = 0; j < transitions.size(); ++j)
        ss << transitions[j]->name() << ", \n\t";
      hppDout(info, ss.str());
#endif  // HPP_DEBUG
      if (optData_) {
        delete optData_;
        optData_ = nullptr;
      }
      optData_ = new OptimizationData(problem(), q1, q2, transitions,
                                      goalDefinedByConstraints_);

      if (buildOptimizationProblem(transitions)) {
        lastBuiltTransitions_ = transitions;
        if (nTryConfigList_ > 0 ||
            analyseOptimizationProblem(transitions, problem())) {
          if (solveOptimizationProblem()) {
            core::Configurations_t path = getConfigList();
            hppDout(warning,
                    " Solution " << idxSol << ": solved configurations list");
            return path;
          } else {
            hppDout(info, " Failed solution " << idxSol
                                              << " at step 5 (solve opt pb)");
          }
        } else {
          hppDout(info, " Failed solution " << idxSol
                                            << " at step 4 (analyse opt pb)");
        }
      } else {
        hppDout(info,
                " Failed solution " << idxSol << " at step 3 (build opt pb)");
      }
      transitions = getTransitionList(d, ++idxSol);
      // reset of the number of tries for a sequence
      // nTryConfigList_ = 0;
    }
  }
  core::Configurations_t empty_path;
  empty_path.push_back(q1);
  return empty_path;
}

void StatesPathFinder::reset() {
  // when debugging, if we want to start from a certain transition list,
  // we can set it here
  graphData_->idxSol = 0;
  if (optData_) {
    delete optData_;
    optData_ = nullptr;
  }
  lastBuiltTransitions_.clear();
  idxConfigList_ = 0;
  nTryConfigList_ = 0;
}

void StatesPathFinder::startSolve() {
  PathPlanner::startSolve();
  assert(problem_);
  q1_ = problem_->initConfig();
  assert(q1_.size() > 0);

  // core::PathProjectorPtr_t pathProjector
  //   (core::pathProjector::Progressive::create(inStateProblem_, 0.01));
  // inStateProblem_->pathProjector(pathProjector);
  inStateProblem_->pathProjector(problem_->pathProjector());
  const graph::GraphPtr_t& graph(problem_->constraintGraph());
  graphData_.reset(new GraphSearchData());
  GraphSearchData& d = *graphData_;
  d.s1 = graph->getState(q1_);
  d.maxDepth = problem_->getParameter("StatesPathFinder/maxDepth").intValue();

  d.queue1.push_back(d.addInitState());
  d.queueIt = d.queue1.size();

#ifdef HPP_DEBUG
  // Print out the names of all the states in graph in debug mode
  States_t allStates = graph->stateSelector()->getStates();
  hppDout(info, "Constraint graph has " << allStates.size() << " nodes");
  for (auto state : allStates) {
    hppDout(info, "State: id = " << state->id() << " name = \"" << state->name()
                                 << "\"");
  }
  hppDout(info,
          "Constraint graph has " << graph->nbComponents() << " components");
#endif
  // Detect whether the goal is defined by a configuration or by a
  // set of constraints
  ProblemTargetPtr_t target(problem()->target());
  GoalConfigurationsPtr_t goalConfigs(
      HPP_DYNAMIC_PTR_CAST(GoalConfigurations, target));
  if (goalConfigs) {
    goalDefinedByConstraints_ = false;
    core::Configurations_t q2s = goalConfigs->configurations();
    if (q2s.size() != 1) {
      std::ostringstream os;
      os << "StatesPathFinder accept one and only one goal "
            "configuration, ";
      os << q2s.size() << " provided.";
      throw std::logic_error(os.str().c_str());
    }
    q2_ = q2s[0];
    d.s2.push_back(graph->getState(q2_));
  } else {
    TaskTargetPtr_t taskTarget(HPP_DYNAMIC_PTR_CAST(TaskTarget, target));
    if (!taskTarget) {
      std::ostringstream os;
      os << "StatesPathFinder only accept goal defined as "
            "either a configuration or a set of constraints.";
      throw std::logic_error(os.str().c_str());
    }
    assert(q2_.size() == 0);
    goalDefinedByConstraints_ = true;
    goalConstraints_ = taskTarget->constraints();
    hppDout(info, "goal defined as a set of constraints");

    int maxNumConstr = -1;
    for (StatePtr_t state : graph->stateSelector()->getStates()) {
      NumericalConstraints_t stateConstr = state->numericalConstraints();
      int numConstr = 0;
      for (auto goalConstraint : goalConstraints_) {
        if (std::find(stateConstr.begin(), stateConstr.end(), goalConstraint) !=
            stateConstr.end()) {
          ++numConstr;
          hppDout(info, "State \"" << state->name() << "\" "
                                   << "has goal constraint: \""
                                   << goalConstraint->function().name()
                                   << "\"");
        }
      }
      if (numConstr > maxNumConstr) {
        d.s2.clear();
        d.s2.push_back(state);
        maxNumConstr = numConstr;
      } else if (numConstr == maxNumConstr) {
        d.s2.push_back(state);
      }
    }
    d.idxSol = 0;
  }
  reset();
}

void StatesPathFinder::oneStep() {
  if (idxConfigList_ == 0) {
    // TODO: accommodate when goal is a set of constraints
    assert(q1_.size() > 0);
    configList_ = computeConfigList(q1_, q2_);
    if (configList_.size() <= 1) {  // max depth reached
      reset();
      throw core::path_planning_failed("Maximal depth reached.");
    }
  }
  size_t& idxSol = graphData_->idxSol;
  Configuration_t q1, q2;
  if (idxConfigList_ >= configList_.size() - 1) {
    reset();
    throw core::path_planning_failed(
        "Final config reached but goal is not reached.");
  }
  q1 = configList_[idxConfigList_];
  q2 = configList_[idxConfigList_ + 1];
  const graph::EdgePtr_t& edge(lastBuiltTransitions_[idxConfigList_]);
  // Copy edge constraints
  core::ConstraintSetPtr_t constraints(HPP_DYNAMIC_PTR_CAST(
      core::ConstraintSet, edge->pathConstraint()->copy()));
  // Initialize right hand side
  constraints->configProjector()->rightHandSideFromConfig(q1);
  assert(constraints->isSatisfied(q2));
  inStateProblem_->constraints(constraints);
  inStateProblem_->pathValidation(edge->pathValidation());
  inStateProblem_->steeringMethod(edge->steeringMethod());
  inStateProblem_->initConfig(q1);
  inStateProblem_->resetGoalConfigs();
  inStateProblem_->addGoalConfig(q2);

  /// use inner state planner to plan path between two configurations.
  /// these configs lie on same leaf (same RHS wrt edge constraints)
  /// eg consecutive configs in the solved config list
  core::PathPlannerPtr_t inStatePlanner(
      core::DiffusingPlanner::create(inStateProblem_));
  inStatePlanner->maxIterations(
      problem_->getParameter("StatesPathFinder/innerPlannerMaxIterations")
          .intValue());
  value_type innerPlannerTimeout =
      problem_->getParameter("StatesPathFinder/innerPlannerTimeOut")
          .floatValue();
  // only set timeout if it is more than 0. default is infinity
  if (innerPlannerTimeout > 0.) {
    inStatePlanner->timeOut(innerPlannerTimeout);
  }
  hppDout(info,
          "calling InStatePlanner_.solve for transition " << idxConfigList_);
  core::PathVectorPtr_t path;
  try {
    path = inStatePlanner->solve();
    for (std::size_t r = 0; r < path->numberPaths() - 1; r++)
      assert(path->pathAtRank(r)->end() == path->pathAtRank(r + 1)->initial());
    idxConfigList_++;
    if (idxConfigList_ == configList_.size() - 1) {
      hppDout(
          warning, "Solution "
                       << idxSol << ": Success"
                       << "\n-----------------------------------------------");
    }
  } catch (const core::path_planning_failed& error) {
    std::ostringstream oss;
    oss << "Error " << error.what() << "\n";
    oss << "Solution " << idxSol << ": Failed to build path at edge "
        << idxConfigList_ << ": ";
    oss << lastBuiltTransitions_[idxConfigList_]->name();
    hppDout(warning, oss.str());

    idxConfigList_ = 0;
    // Retry nTryMax times to build another solution for the same states list
    size_type nTryMax =
        problem_->getParameter("StatesPathFinder/maxTriesBuildPath").intValue();
    if (++nTryConfigList_ >= nTryMax) {
      nTryConfigList_ = 0;
      idxSol++;
    }
  }
  roadmap()->merge(inStatePlanner->roadmap());
  // if (path) {
  //   core::PathOptimizerPtr_t inStateOptimizer
  //     (core::pathOptimization::RandomShortcut::create(inStateProblem_));
  //   core::PathVectorPtr_t opt = inStateOptimizer->optimize(path);
  //   roadmap()->insertPathVector(opt, true);
  // }
}

void StatesPathFinder::tryConnectInitAndGoals() {
  GraphSearchData& d = *graphData_;
  // if start state is not one of the potential goal states, return
  if (std::find(d.s2.begin(), d.s2.end(), d.s1) == d.s2.end()) {
    return;
  }

  // get the loop edge connecting the initial state to itself
  const graph::Edges_t& loopEdges(
      problem_->constraintGraph()->getEdges(d.s1, d.s1));
  // check that there is 1 loop edge
  assert(loopEdges.size() == 1);
  // add the loop transition as transition list
  GraphSearchData::state_with_depth_ptr_t _state = d.queue1.front();
  GraphSearchData::state_with_depth_ptr_t _endState =
      d.addParent(_state, loopEdges[0]);
  d.solutions.push_back(_endState);

  // try connecting initial and final configurations directly
  if (!goalDefinedByConstraints_) PathPlanner::tryConnectInitAndGoals();
}

std::vector<std::string> StatesPathFinder::constraintNamesFromSolverAtWaypoint(
    std::size_t wp) {
  assert(wp > 0 && wp <= optData_->solvers.size());
  constraints::solver::BySubstitution& solver(optData_->solvers[wp - 1]);
  std::vector<std::string> ans;
  for (std::size_t i = 0; i < solver.constraints().size(); i++)
    ans.push_back(solver.constraints()[i]->function().name());
  return ans;
}

std::vector<std::string> StatesPathFinder::lastBuiltTransitions() const {
  std::vector<std::string> ans;
  for (const EdgePtr_t& edge : lastBuiltTransitions_)
    ans.push_back(edge->name());
  return ans;
}

using core::Parameter;
using core::ParameterDescription;

HPP_START_PARAMETER_DECLARATION(StatesPathFinder)
core::Problem::declareParameter(ParameterDescription(
    Parameter::INT, "StatesPathFinder/maxDepth",
    "Maximum number of transitions to look for.",
    Parameter((size_type)std::numeric_limits<int>::max())));
core::Problem::declareParameter(ParameterDescription(
    Parameter::INT, "StatesPathFinder/maxIteration",
    "Maximum number of iterations of the Newton Raphson algorithm.",
    Parameter((size_type)60)));
core::Problem::declareParameter(ParameterDescription(
    Parameter::FLOAT, "StatesPathFinder/errorThreshold",
    "Error threshold of the Newton Raphson algorithm."
    "Should be at most the same as error threshold in constraint graph",
    Parameter(1e-4)));
core::Problem::declareParameter(ParameterDescription(
    Parameter::INT, "StatesPathFinder/nTriesUntilBacktrack",
    "Number of tries when sampling configurations before backtracking"
    "in function solveOptimizationProblem.",
    Parameter((size_type)3)));
core::Problem::declareParameter(ParameterDescription(
    Parameter::INT, "StatesPathFinder/maxTriesCollisionAnalysis",
    "Number of solve tries before stopping the collision analysis,"
    "before the actual solving part."
    "Set to 0 to skip this part of the algorithm.",
    Parameter((size_type)100)));
core::Problem::declareParameter(ParameterDescription(
    Parameter::INT, "StatesPathFinder/maxTriesBuildPath",
    "Number of solutions with a given states list to try to build a"
    "continuous path from, before skipping to the next states list",
    Parameter((size_type)5)));
core::Problem::declareParameter(ParameterDescription(
    Parameter::FLOAT, "StatesPathFinder/innerPlannerTimeOut",
    "This will set ::timeOut accordingly in the inner"
    "planner used for building a path after intermediate"
    "configurations have been found."
    "If set to 0, no timeout: only maxIterations will be used to stop"
    "the innerPlanner if it does not find a path.",
    Parameter(2.0)));
core::Problem::declareParameter(ParameterDescription(
    Parameter::INT, "StatesPathFinder/innerPlannerMaxIterations",
    "This will set ::maxIterations accordingly in the inner"
    "planner used for building a path after intermediate"
    "configurations have been found",
    Parameter((size_type)1000)));
HPP_END_PARAMETER_DECLARATION(StatesPathFinder)
}  // namespace pathPlanner
}  // namespace manipulation
}  // namespace hpp
