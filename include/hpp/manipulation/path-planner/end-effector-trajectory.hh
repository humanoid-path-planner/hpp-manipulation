// Copyright (c) 2019 CNRS
// Authors: Joseph Mirabel
//
// This file is part of hpp-manipulation
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
// hpp-manipulation  If not, see
// <http://www.gnu.org/licenses/>.


#ifndef HPP_MANIPULATION_PATH_PLANNER_END_EFFECTOR_TRAJECTORY_HH
# define HPP_MANIPULATION_PATH_PLANNER_END_EFFECTOR_TRAJECTORY_HH

# include <hpp/manipulation/config.hh>
# include <hpp/manipulation/fwd.hh>

# include <pinocchio/spatial/se3.hpp>

# include <hpp/pinocchio/frame.hh>
# include <hpp/core/path-planner.hh>

namespace hpp {
  namespace manipulation {
    namespace pathPlanner {
      class HPP_MANIPULATION_DLLAPI IkSolverInitialization
      {
        public:
          typedef std::vector<Configuration_t> Configurations_t;

          Configurations_t solve (vectorIn_t target)
          {
            return impl_solve (target);
          }

        protected:
          virtual Configurations_t impl_solve (vectorIn_t target) = 0;
      };
      typedef boost::shared_ptr<IkSolverInitialization> IkSolverInitializationPtr_t;

      HPP_PREDEF_CLASS (EndEffectorTrajectory);
      typedef boost::shared_ptr<EndEffectorTrajectory> EndEffectorTrajectoryPtr_t;

      class HPP_MANIPULATION_DLLAPI EndEffectorTrajectory : public core::PathPlanner
      {
      public:
        /// Return shared pointer to new instance
        /// \param problem the path planning problem
        static EndEffectorTrajectoryPtr_t create (const core::Problem& problem);
        /// Return shared pointer to new instance
        /// \param problem the path planning problem
        /// \param roadmap previously built roadmap
        static EndEffectorTrajectoryPtr_t createWithRoadmap
          (const core::Problem& problem, const core::RoadmapPtr_t& roadmap);

        /// Initialize the problem resolution
        ///  \li call parent implementation
        ///  \li get number nodes in problem parameter map
        virtual void startSolve ();

        /// One step of the algorithm
        virtual void oneStep ();

        /// Get the number of random configurations shoot (after using init
        /// config) in order to generate the initial config of the final path.
        int nRandomConfig () const
        { return nRandomConfig_; }

        void nRandomConfig (int n)
        {
          assert (n >= 0);
          nRandomConfig_ = n;
        }

        /// Number of steps to generate goal config (successive projections).
        int nDiscreteSteps () const
        { return nDiscreteSteps_; }

        void nDiscreteSteps (int n)
        {
          assert (n > 0);
          nDiscreteSteps_ = n;
        }

        /// If enabled, only add one solution to the roadmap.
        /// Otherwise add all solution.
        void checkFeasibilityOnly (bool enable);

        bool checkFeasibilityOnly () const
        {
          return feasibilityOnly_;
        }

        void ikSolverInitialization (IkSolverInitializationPtr_t solver)
        {
          ikSolverInit_ = solver;
        }

        void tryConnectInitAndGoals ();

      protected:
        /// Protected constructor
        /// \param problem the path planning problem
        EndEffectorTrajectory (const core::Problem& problem);
        /// Protected constructor
        /// \param problem the path planning problem
        /// \param roadmap previously built roadmap
        EndEffectorTrajectory (const core::Problem& problem, const core::RoadmapPtr_t& roadmap);
        /// Store weak pointer to itself
        void init (const EndEffectorTrajectoryWkPtr_t& weak);

      private:
        std::vector<core::Configuration_t> configurations(const core::Configuration_t& q_init);

        /// Weak pointer to itself
        EndEffectorTrajectoryWkPtr_t weak_;
        /// Number of random config.
        int nRandomConfig_;
        /// Number of steps to generate goal config.
        int nDiscreteSteps_;
        /// Ik solver initialization. An external Ik solver can be plugged here.
        IkSolverInitializationPtr_t ikSolverInit_;
        /// Feasibility
        bool feasibilityOnly_;
      }; // class EndEffectorTrajectory
    } // namespace pathPlanner
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_PATH_PLANNER_END_EFFECTOR_TRAJECTORY_HH
