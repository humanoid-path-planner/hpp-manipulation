// Copyright (c) 2017, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr),
//          Florent Lamiraux (florent.lamiraux@laas.fr)
//          Alexandre Thiault (athiault@laas.fr)
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
# define HPP_MANIPULATION_PATH_PLANNER_STATES_PATH_FINDER_HH

# include <hpp/core/fwd.hh>
# include <hpp/core/path.hh>

# include <hpp/core/projection-error.hh>
# include <hpp/core/config-projector.hh>

# include <hpp/core/validation-report.hh>
# include <hpp/core/config-validations.hh>

# include <hpp/core/path-planner.hh>

# include <hpp/manipulation/config.hh>
# include <hpp/manipulation/fwd.hh>
# include <hpp/manipulation/problem.hh>

namespace hpp {
  namespace manipulation {
    namespace pathPlanner {

      /// \addtogroup path_planner
      /// \{

      /// Optimization-based path planning method.
      ///
      /// #### Sketch of the method
      ///
      /// Given two configuration \f$ (q_1,q_2) \f$, this class formulates and
      /// solves the problem as follows.
      /// - Compute the corresponding states \f$ (s_1, s_2) \f$.
      /// - For a each path \f$ (e_0, ... e_n) \f$ of length not more than
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
      /// - \f$ p_i \f$ is in state between \f$ (e_{i-1}, e_i) \f$, (\ref StateFunction)
      /// - \f$ (p_i, p_{i+1}) \f$ are reachable with transition \f$ e_i \f$ (\ref EdgeFunction).
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
      ///   configuraion, and detects if a collision is certain to block any attemps
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
      class HPP_MANIPULATION_DLLAPI StatesPathFinder : public core::PathPlanner
      {

        public:
          struct OptimizationData;

        virtual ~StatesPathFinder () {};

          static StatesPathFinderPtr_t create (
            const core::ProblemConstPtr_t& problem);
            
          static StatesPathFinderPtr_t createWithRoadmap (
            const core::ProblemConstPtr_t& problem,
            const core::RoadmapPtr_t& roadmap);

          StatesPathFinderPtr_t copy () const;
          
          core::ProblemConstPtr_t problem() const
          {
            return problem_;
          }

          /// create a vector of configurations between two configurations
          /// \return a Configurations_t from q1 to q2 if found. An empty
          /// vector if a path could not be built.
          core::Configurations_t computeConfigList (ConfigurationIn_t q1,
                                          ConfigurationIn_t q2);

          // access functions for Python interface
          std::vector<std::string> constraintNamesFromSolverAtWaypoint
            (std::size_t wp);
          std::vector<std::string> lastBuiltTransitions() const;
          std::string displayConfigsSolved () const;
          bool buildOptimizationProblemFromNames(std::vector<std::string> names);

          // Substeps of method solveOptimizationProblem
          void initWPRandom(std::size_t wp);
          void initWPNear(std::size_t wp);
          void initWP(std::size_t wp, ConfigurationIn_t q);
          int solveStep(std::size_t wp);
          Configuration_t configSolved (std::size_t wp) const;

          /// Step 7 of the algorithm
          core::PathVectorPtr_t pathFromConfigList (std::size_t i) const;

          /// deletes from memory the latest working states list, which is used to
          /// resume finding solutions from that list in case of failure at a
          /// later step.
          void reset();
          core::PathVectorPtr_t buildPath (ConfigurationIn_t q1, ConfigurationIn_t q2);

          virtual void startSolve();
          virtual void oneStep();
          virtual core::PathVectorPtr_t solve ();

        protected:
          StatesPathFinder (const core::ProblemConstPtr_t& problem,
                const core::RoadmapPtr_t&) :
            PathPlanner(problem),
            problem_ (HPP_STATIC_PTR_CAST(const manipulation::Problem, problem)),
            sameRightHandSide_ (), weak_ ()
          {
            gatherGraphConstraints ();
          }

          StatesPathFinder (const StatesPathFinder& other) :
            PathPlanner(other.problem_),
            problem_ (other.problem_), constraints_ (), index_ (other.index_),
            sameRightHandSide_ (other.sameRightHandSide_),  weak_ ()
          {}

          void init (StatesPathFinderWkPtr_t weak)
          {
            weak_ = weak;
          }

        private:
          typedef constraints::solver::BySubstitution Solver_t;
          struct GraphSearchData;

          /// Gather constraints of all edges
          void gatherGraphConstraints ();

          /// Step 1 of the algorithm
          /// \return whether the max depth was reached.
          bool findTransitions (GraphSearchData& data) const;

          /// Step 2 of the algorithm
          graph::Edges_t getTransitionList (const GraphSearchData& data, const std::size_t& i) const;

          /// Step 3 of the algorithm
          bool contains (const Solver_t& solver, const ImplicitPtr_t& c) const;
          bool checkConstantRightHandSide (size_type index);
          bool buildOptimizationProblem (const graph::Edges_t& transitions);

          /// Step 4 of the algorithm
          void preInitializeRHS(std::size_t j, Configuration_t& q);
          bool analyseOptimizationProblem (const graph::Edges_t& transitions);

          /// Step 5 of the algorithm
          void initializeRHS (std::size_t j);
          bool solveOptimizationProblem ();

          /// Step 6 of the algorithm
          core::Configurations_t buildConfigList () const;

          /// Functions used in assert statements
          bool checkWaypointRightHandSide (std::size_t ictr, std::size_t jslv) const;
          bool checkSolverRightHandSide (std::size_t ictr, std::size_t jslv) const;
          bool checkWaypointRightHandSide (std::size_t jslv) const;
          bool checkSolverRightHandSide (std::size_t jslv) const;

          void displayRhsMatrix ();
          void displayStatusMatrix (const graph::Edges_t& transitions);

          /// A pointer to the manipulation problem
          ProblemConstPtr_t problem_;

          /// Vector of parameterizable edge numerical constraints
          NumericalConstraints_t constraints_;
          /// Map of indexes in constraints_
          std::map < std::string, std::size_t > index_;

          /// associative map that stores pairs of constraints of the form
          /// (constraint, constraint/hold)
          std::map <ImplicitPtr_t, ImplicitPtr_t> sameRightHandSide_;

          mutable OptimizationData* optData_ = nullptr;
          std::size_t idxSol_ = 0;
          graph::Edges_t lastBuiltTransitions_;

          bool skipColAnalysis_ = false;

          // Variables used across several calls to oneStep
          ConfigurationPtr_t q1_, q2_;
          core::Configurations_t configList_;
          std::size_t idxConfigList_ = 0;
          size_type nTryConfigList_ = 0;
          InStatePathPtr_t planner_;
          core::PathVectorPtr_t solution_;
          bool solved_ = false, interrupt_ = false;

          /// Weak pointer to itself
          StatesPathFinderWkPtr_t weak_;

      }; // class StatesPathFinder
      /// \}
      
    } // namespace pathPlanner
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_PATH_PLANNER_STATES_PATH_FINDER_HH
