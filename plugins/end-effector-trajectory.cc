// Copyright (c) 2019, Joseph Mirabel
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

#include <hpp/core/plugin.hh>
#include <hpp/core/problem-solver.hh>

#include <hpp/manipulation/steering-method/end-effector-trajectory.hh>
#include <hpp/manipulation/path-planner/end-effector-trajectory.hh>

namespace hpp {
  namespace manipulation {
    class EndEffectorTrajectoryPlugin : public core::ProblemSolverPlugin
    {
      public:
        EndEffectorTrajectoryPlugin ()
          : ProblemSolverPlugin ("EndEffectorTrajectoryPlugin", "0.0")
        {}

      protected:
        virtual bool impl_initialize (core::ProblemSolverPtr_t ps)
        {
          ps->pathPlanners.add ("EndEffectorTrajectory",
              pathPlanner::EndEffectorTrajectory::createWithRoadmap);
          ps->steeringMethods.add ("EndEffectorTrajectory",
              steeringMethod::EndEffectorTrajectory::create);
          return true;
        }
    };
  } // namespace manipulation
} // namespace hpp

HPP_CORE_DEFINE_PLUGIN(hpp::manipulation::EndEffectorTrajectoryPlugin)
