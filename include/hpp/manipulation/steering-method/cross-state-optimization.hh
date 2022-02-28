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

#ifndef HPP_MANIPULATION_STEERING_METHOD_CROSS_STATE_OPTIMIZATION_HH
# define HPP_MANIPULATION_STEERING_METHOD_CROSS_STATE_OPTIMIZATION_HH

# include <hpp/core/steering-method.hh>
# include <hpp/core/config-projector.hh>

# include <hpp/manipulation/config.hh>
# include <hpp/manipulation/fwd.hh>
# include <hpp/manipulation/problem.hh>
# include <hpp/manipulation/steering-method/fwd.hh>
# include <hpp/manipulation/steering-method/graph.hh>

namespace hpp {
  namespace manipulation {
    namespace steeringMethod {
      /// \addtogroup steering_method
      /// \{

      /// Optimization-based steering method.
      ///
      /// #### Sketch of the method
      ///
      /// Given two configuration \f$ (q_1,q_2) \f$, this class formulates and
      /// solves the problem as follows.
      /// - Compute the corresponding states \f$ (s_1, s_2) \f$.
      /// - For a each path \f$ (e_0, ... e_n) \f$ of length not more than
      ///   parameter "CrossStateOptimization/maxDepth" between
      ///   \f$ (s_1, s_2)\f$ in the constraint graph, do:
      ///   - define \f$ n-1 \f$ intermediate configuration \f$ p_i \f$,
      ///   - initialize the optimization problem, as explained below,
      ///   - solve the optimization problem, which gives \f$ p^*_i \f$,
      ///   - in case of failure, continue the loop.
      ///   - call the Edge::build of each \f$ e_j \f$ for each consecutive
      ///     \f$ (p^*_i, p^*_{i+1}) \f$.
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
      /// - method CrossStateOptimization::solveOptimizationProblem loops over
      ///   the waypoint solvers, solves for each waypoint after
      ///   initializing the right hand sides with the proper values.
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
      class HPP_MANIPULATION_DLLAPI CrossStateOptimization :
        public SteeringMethod
      {
        public:
          struct OptimizationData;

          static CrossStateOptimizationPtr_t create
	    (const ProblemConstPtr_t& problem);

          /// \warning core::Problem will be casted to Problem
          static CrossStateOptimizationPtr_t create
            (const core::ProblemConstPtr_t& problem);

          template <typename T>
            static CrossStateOptimizationPtr_t create
            (const core::ProblemConstPtr_t& problem);

          core::SteeringMethodPtr_t copy () const;

        protected:
          CrossStateOptimization (const ProblemConstPtr_t& problem) :
            SteeringMethod (problem),
            sameRightHandSide_ ()
          {
            gatherGraphConstraints ();
          }

          CrossStateOptimization (const CrossStateOptimization& other) :
            SteeringMethod (other), constraints_ (other.constraints_),
            index_ (other.index_), sameRightHandSide_
            (other.sameRightHandSide_), weak_ ()
          {}

          core::PathPtr_t impl_compute (ConfigurationIn_t q1, ConfigurationIn_t q2) const;

          void init (CrossStateOptimizationWkPtr_t weak)
          {
            SteeringMethod::init (weak);
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
          graph::Edges_t getTransitionList (GraphSearchData& data, const std::size_t& i) const;

          /// Step 3 of the algorithm
          bool buildOptimizationProblem
            (OptimizationData& d, const graph::Edges_t& transitions) const;

          /// Step 4 of the algorithm
          bool solveOptimizationProblem (OptimizationData& d) const;

          bool checkConstantRightHandSide (OptimizationData& d,
                                           size_type index) const;

          core::PathVectorPtr_t buildPath (OptimizationData& d, const graph::Edges_t& edges) const;

          bool contains (const Solver_t& solver, const ImplicitPtr_t& c) const;

          /// Vector of parameterizable edge numerical constraints
          NumericalConstraints_t constraints_;
          /// Map of indexes in constraints_
          std::map < std::string, std::size_t > index_;

          /// associative map that stores pairs of constraints of the form
          /// (constraint, constraint/hold)
          std::map <ImplicitPtr_t, ImplicitPtr_t> sameRightHandSide_;

          /// Weak pointer to itself
          CrossStateOptimizationWkPtr_t weak_;
      }; // class CrossStateOptimization
      /// \}

      template <typename T>
        CrossStateOptimizationPtr_t CrossStateOptimization::create
        (const core::ProblemConstPtr_t& problem)
      {
        CrossStateOptimizationPtr_t gsm = CrossStateOptimization::create
	  (problem);
        gsm->innerSteeringMethod (T::create (problem));
        return gsm;
      }
    } // namespace steeringMethod
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_STEERING_METHOD_CROSS_STATE_OPTIMIZATION_HH
