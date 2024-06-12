// Copyright (c) 2017, Joseph Mirabel
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

#ifndef HPP_MANIPULATION_PATH_PLANNER_STATES_PATH_FINDER_HH
#define HPP_MANIPULATION_PATH_PLANNER_STATES_PATH_FINDER_HH

#include <hpp/core/config-projector.hh>
#include <hpp/core/config-validations.hh>
#include <hpp/core/fwd.hh>
#include <hpp/core/path-planner.hh>
#include <hpp/core/path.hh>
#include <hpp/core/projection-error.hh>
#include <hpp/core/validation-report.hh>
#include <hpp/manipulation/config.hh>
#include <hpp/manipulation/fwd.hh>
#include <hpp/manipulation/problem.hh>

namespace hpp {
namespace manipulation {
namespace pathPlanner {

/// \addtogroup path_planner
/// \{

/// Optimization-based path planning method.
///
/// #### Sketch of the method
///
/// Given two configurations \f$ (q_1,q_2) \f$, this class formulates and
/// solves the problem as follows.
/// - Compute the corresponding states \f$ (s_1, s_2) \f$.
/// - For each path \f$ (e_0, ... e_n) \f$ of length not more than
///   parameter "StatesPathFinder/maxDepth" between
///   \f$ (s_1, s_2)\f$ in the constraint graph, do:
///   - define \f$ n-1 \f$ intermediate configuration \f$ p_i \f$,
///   - initialize the optimization problem, as explained below,
///   - solve the optimization problem, which gives \f$ p^*_i \f$,
///   - in case of failure, continue the loop.
///
/// #### Problem formulation
/// Find \f$ (p_i) \f$ such that:
/// - \f$ p_0 = q_1 \f$,
/// - \f$ p_{n+1} = q_2 \f$,
/// - \f$ p_i \f$ is in state between \f$ (e_{i-1}, e_i) \f$, (\ref
/// StateFunction)
/// - \f$ (p_i, p_{i+1}) \f$ are reachable with transition \f$ e_i \f$ (\ref
/// EdgeFunction).
///
/// #### Problem resolution
///
/// One solver (hpp::constraints::solver::BySubstitution) is created
/// for each waypoint \f$p_i\f$.
/// - method buildOptimizationProblem builds a matrix the rows of which
///   are the parameterizable numerical constraints present in the
///   graph, and the columns of which are the waypoints. Each value in the
///   matrix defines the status of each constraint right hand side for
///   this waypoint, among {absent from the solver,
///                         equal to value for previous waypoint,
///                         equal to value for start configuration,
///                         equal to value for end configuration}.
/// - method analyseOptimizationProblem loops over the waypoint solvers,
///   tests what happens when solving each waypoint after initializing
///   only the right hand sides that are equal to the initial or goal
///   configuration, and detects if a collision is certain to block any attempts
///   to solve the problem in the solveOptimizationProblem step.
/// - method solveOptimizationProblem tries to solve for each waypoint after
///   initializing the right hand sides with the proper values, backtracking
///   to the previous waypoint if the solving failed or a collision is
///   detected a number of times set from the parameter
///   "StatesPathFinder/nTriesUntilBacktrack". If too much backtracking
///   occurs, the method can eventually return false.
/// - eventually method buildPath build paths between waypoints with
///   the constraints of the transition in which the path lies.
///
/// #### Current status
///
/// The method has been successfully tested with romeo holding a placard
/// and the construction set benchmarks. The result is satisfactory
/// except between pregrasp and grasp waypoints that may be far
/// away from each other if the transition between those state does
/// not contain the grasp complement constraint. The same holds
/// between placement and pre-placement.
class HPP_MANIPULATION_DLLAPI StatesPathFinder : public core::PathPlanner {
 public:
  struct OptimizationData;

  virtual ~StatesPathFinder() {};

  static StatesPathFinderPtr_t create(const core::ProblemConstPtr_t& problem);

  static StatesPathFinderPtr_t createWithRoadmap(
      const core::ProblemConstPtr_t& problem,
      const core::RoadmapPtr_t& roadmap);

  StatesPathFinderPtr_t copy() const;

  /// create a vector of configurations between two configurations
  /// \param q1 initial configuration
  /// \param q2 pointer to final configuration, NULL if goal is
  /// defined as a set of constraints
  /// \return a Configurations_t from q1 to q2 if found. An empty
  /// vector if a path could not be built.
  core::Configurations_t computeConfigList(ConfigurationIn_t q1,
                                           ConfigurationIn_t q2);

  // access functions for Python interface
  std::vector<std::string> constraintNamesFromSolverAtWaypoint(std::size_t wp);
  std::vector<std::string> lastBuiltTransitions() const;
  std::string displayConfigsSolved() const;
  bool buildOptimizationProblemFromNames(std::vector<std::string> names);

  // Substeps of method solveOptimizationProblem
  void initWPRandom(std::size_t wp);
  void initWPNear(std::size_t wp);
  void initWP(std::size_t wp, ConfigurationIn_t q);
  /// Status of the step to solve for a particular waypoint
  enum SolveStepStatus {
    /// Valid solution (no collision)
    VALID_SOLUTION,
    /// Bad solve status, no solution from the solver
    NO_SOLUTION,
    /// Solution has collision in edge leading to the waypoint
    COLLISION_BEFORE,
    /// Solution has collision in edge going from the waypoint
    COLLISION_AFTER,
  };

  SolveStepStatus solveStep(std::size_t wp);

  /// deletes from memory the latest working states list, which is used to
  /// resume finding solutions from that list in case of failure at a
  /// later step.
  void reset();

  virtual void startSolve();
  virtual void oneStep();
  /// when both initial state is one of potential goal states,
  /// try connecting them directly
  virtual void tryConnectInitAndGoals();

 protected:
  StatesPathFinder(const core::ProblemConstPtr_t& problem,
                   const core::RoadmapPtr_t&);
  StatesPathFinder(const StatesPathFinder& other);

  void init(StatesPathFinderWkPtr_t weak) { weak_ = weak; }

 private:
  typedef constraints::solver::BySubstitution Solver_t;
  struct GraphSearchData;

  /// Gather constraints of all edges
  void gatherGraphConstraints();

  /// Step 1 of the algorithm
  /// \return whether the max depth was reached.
  bool findTransitions(GraphSearchData& data) const;

  /// Step 2 of the algorithm
  graph::Edges_t getTransitionList(const GraphSearchData& data,
                                   const std::size_t& i) const;

  /// Step 3 of the algorithm

  // check that the solver either contains exactly same constraint
  // or a constraint with similar parametrizable form
  // constraint/both and constraint/complement
  bool contains(const Solver_t& solver, const ImplicitPtr_t& c) const;

  // check that the solver either contains exactly same constraint
  // or a stricter version of the constraint
  // constraint/both stricter than constraint and stricter than
  // constraint/complement
  bool containsStricter(const Solver_t& solver, const ImplicitPtr_t& c) const;
  bool checkConstantRightHandSide(size_type index);
  bool buildOptimizationProblem(const graph::Edges_t& transitions);

  /// Step 4 of the algorithm
  void preInitializeRHS(std::size_t j, Configuration_t& q);
  bool analyseOptimizationProblem(const graph::Edges_t& transitions,
                                  core::ProblemConstPtr_t _problem);

  /// Step 5 of the algorithm
  void initializeRHS(std::size_t j);
  bool solveOptimizationProblem();

  // Data structure used to store a constraint right hand side as value and its
  // name as key, both in form of hash numbers (so that names and rhs of two
  // constraints can be easily merge). Exemple : ConstraintMap_t map =
  // {{nameStringHash, rhsVectorHash}}; With rhsVectorHash the hash of a
  // vector_t of rights hand side constraints with hashRHS, and nameStringHash
  // the hash of a std::string - obtained for instance with std::hash.
  typedef std::unordered_map<size_t, size_t> ConstraintMap_t;  // map (name,
                                                               // rhs)

  /// @brief Get a configuration in accordance with the statuts matrix at a step
  /// j for the constraint i
  /// @param i number of the constraint in the status matrix
  /// @param j step of the potential solution (index of a waypoint)
  /// @return a configuration Configuration_t which follows the status matrix
  /// indication at the given indices
  Configuration_t getConfigStatus(size_type i, size_type j) const;

  /// @brief Get the right hand side of a constraint w.r.t a set configuration
  /// for this constraint
  /// @param constraint the constraint to compute the right hand side of
  /// @param q the configuration in which the constraint is set
  /// @return a right hand side vector_t
  vector_t getConstraintRHS(ImplicitPtr_t constraint, Configuration_t q) const;

  /// @brief Hash a vector of right hand side into a long unsigned integer
  /// @param rhs the right hand side vector vector_t
  /// @return a size_t integer hash
  size_t hashRHS(vector_t rhs) const;

  /// @brief Check if a solution (a list of transition) contains impossible to
  /// solve steps due to inevitable collisions
  /// @param pairMap The ConstraintMap_tf table of pairs of incompatibles
  /// constraints
  /// @param constraintMap The hasmap table of constraints which are in pairMap
  /// @return a bool which is true is there is no impossible to solve steps,
  /// true otherwise
  bool checkSolvers(ConstraintMap_t const& pairMap,
                    ConstraintMap_t const& constraintMap) const;

  /// @brief For a certain step wp during solving check for collision impossible
  /// to solve.
  /// @param pairMap The ConstraintMap_t table of pairs of incompatibles
  /// constraints
  /// @param constraintMap The hasmap table of constraints which are in pairMap
  /// @param wp The index of the current step
  /// @return a bool which is true if there is no collision or impossible to
  /// solve ones, false otherwise.
  bool saveIncompatibleRHS(ConstraintMap_t& pairMap,
                           ConstraintMap_t& constraintMap, size_type const wp);

  // For a joint get his most, constrained with it, far parent
  core::JointConstPtr_t maximalJoint(size_t const wp, core::JointConstPtr_t a);

  /// Step 6 of the algorithm
  core::Configurations_t getConfigList() const;

  /// Functions used in assert statements
  bool checkWaypointRightHandSide(std::size_t ictr, std::size_t jslv) const;
  bool checkSolverRightHandSide(std::size_t ictr, std::size_t jslv) const;
  bool checkWaypointRightHandSide(std::size_t jslv) const;
  bool checkSolverRightHandSide(std::size_t jslv) const;

  void displayRhsMatrix();
  void displayStatusMatrix(const graph::Edges_t& transitions);

  /// A pointer to the manipulation problem
  ProblemConstPtr_t problem_;
  /// Path planning problem in each leaf.
  core::ProblemPtr_t inStateProblem_;

  /// Vector of parameterizable edge numerical constraints
  NumericalConstraints_t constraints_;
  /// Map of indices in constraints_
  std::map<std::string, std::size_t> index_;

  /// associative map that stores pairs of constraints of the form
  /// (constraint/complement, constraint/hold)
  std::map<ImplicitPtr_t, ImplicitPtr_t> sameRightHandSide_;

  /// associative map that stores pairs of constraints of either form
  /// (constraint, constraint/hold)
  /// or (constraint/complement, constraint/hold)
  std::map<ImplicitPtr_t, ImplicitPtr_t> stricterConstraints_;

  mutable OptimizationData* optData_;
  mutable std::shared_ptr<GraphSearchData> graphData_;
  graph::Edges_t lastBuiltTransitions_;

  /// Constraints defining the goal
  /// For now:
  /// - comparison type Equality is initialized to zero
  /// - if goal constraint is not already present in any graph state,
  ///   it should not require propagating a complement.
  ///   invalid eg: specify the full pose of an object placement or
  ///   object grasp
  NumericalConstraints_t goalConstraints_;
  bool goalDefinedByConstraints_;
  // Variables used across several calls to oneStep
  Configuration_t q1_, q2_;
  core::Configurations_t configList_;
  std::size_t idxConfigList_;
  size_type nTryConfigList_;
  bool solved_, interrupt_;

  /// Weak pointer to itself
  StatesPathFinderWkPtr_t weak_;

};  // class StatesPathFinder
/// \}

}  // namespace pathPlanner
}  // namespace manipulation
}  // namespace hpp

#endif  // HPP_MANIPULATION_PATH_PLANNER_STATES_PATH_FINDER_HH
