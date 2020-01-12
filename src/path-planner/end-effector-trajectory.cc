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

# include <hpp/manipulation/path-planner/end-effector-trajectory.hh>

# include <pinocchio/multibody/data.hpp>

# include <hpp/util/exception-factory.hh>
# include <hpp/pinocchio/util.hh>
# include <hpp/pinocchio/device-sync.hh>
# include <hpp/pinocchio/liegroup-element.hh>

# include <hpp/constraints/differentiable-function.hh>
# include <hpp/constraints/implicit.hh>

# include <hpp/core/config-projector.hh>
# include <hpp/core/config-validations.hh>
# include <hpp/core/configuration-shooter.hh>
# include <hpp/core/path-validation.hh>
# include <hpp/core/problem.hh>
# include <hpp/core/roadmap.hh>
# include <hpp/manipulation/steering-method/end-effector-trajectory.hh>

namespace hpp {
  namespace manipulation {
    namespace pathPlanner {
      typedef manipulation::steeringMethod::EndEffectorTrajectory      SM_t;
      typedef manipulation::steeringMethod::EndEffectorTrajectoryPtr_t SMPtr_t;

      EndEffectorTrajectoryPtr_t EndEffectorTrajectory::create (const core::Problem& problem)
      {
        EndEffectorTrajectoryPtr_t ptr (new EndEffectorTrajectory(problem));
        ptr->init(ptr);
        return ptr;
      }

      EndEffectorTrajectoryPtr_t EndEffectorTrajectory::createWithRoadmap (
          const core::Problem& problem, const core::RoadmapPtr_t& roadmap)
      {
        EndEffectorTrajectoryPtr_t ptr (new EndEffectorTrajectory(problem, roadmap));
        ptr->init(ptr);
        return ptr;
      }

      void EndEffectorTrajectory::tryConnectInitAndGoals ()
      {}

      void EndEffectorTrajectory::startSolve ()
      {
        //core::PathPlanner::startSolve();
        //problem().checkProblem ();
        if (!problem().robot ()) {
          std::string msg ("No device in problem.");
          hppDout (error, msg);
          throw std::runtime_error (msg);
        }

        if (!problem().initConfig ()) {
          std::string msg ("No init config in problem.");
          hppDout (error, msg);
          throw std::runtime_error (msg);
        }

        // Tag init and goal configurations in the roadmap
        roadmap()->resetGoalNodes ();

        SMPtr_t sm (HPP_DYNAMIC_PTR_CAST (SM_t, problem().steeringMethod()));
        if (!sm)
          throw std::invalid_argument ("Steering method must be of type hpp::manipulation::steeringMethod::EndEffectorTrajectory");

        if (!sm->constraints() || !sm->constraints()->configProjector())
          throw std::invalid_argument ("Steering method constraint has no ConfigProjector.");
        core::ConfigProjectorPtr_t constraints (sm->constraints()->configProjector());

        const constraints::ImplicitPtr_t& trajConstraint = sm->trajectoryConstraint ();
        if (!trajConstraint)
          throw std::invalid_argument ("EndEffectorTrajectory has no trajectory constraint.");
        if (!sm->trajectory())
          throw std::invalid_argument ("EndEffectorTrajectory has no trajectory.");

        const core::NumericalConstraints_t& ncs = constraints->numericalConstraints();
        bool ok = false;
        for (std::size_t i = 0; i < ncs.size(); ++i) {
          if (ncs[i] == trajConstraint) {
            ok = true;
            break; // Same pointer
          }
          // Here, we do not check the right hand side on purpose.
          // if (*ncs[i] == *trajConstraint) {
          if (ncs[i]->functionPtr() == trajConstraint->functionPtr()
              && ncs[i]->comparisonType() == trajConstraint->comparisonType()) {
            ok = true;
            // TODO We should only modify the path constraint.
            // However, only the pointers to implicit constraints are copied
            // while we would like the implicit constraints to be copied as well.
            ncs[i]->rightHandSideFunction (sm->trajectory());
            break; // logically identical
          }
        }
        if (!ok) {
          HPP_THROW (std::logic_error, "EndEffectorTrajectory could not find "
              "constraint " << trajConstraint->function());
        }
      }

      void EndEffectorTrajectory::oneStep ()
      {
        SMPtr_t sm (HPP_DYNAMIC_PTR_CAST (SM_t, problem().steeringMethod()));
        if (!sm)
          throw std::invalid_argument ("Steering method must be of type hpp::manipulation::steeringMethod::EndEffectorTrajectory");
        if (!sm->trajectoryConstraint ())
          throw std::invalid_argument ("EndEffectorTrajectory has no trajectory constraint.");
        if (!sm->trajectory())
          throw std::invalid_argument ("EndEffectorTrajectory has no trajectory.");

        if (!sm->constraints() || !sm->constraints()->configProjector())
          throw std::invalid_argument ("Steering method constraint has no ConfigProjector.");
        core::ConfigProjectorPtr_t constraints (sm->constraints()->configProjector());

        core::ConfigValidationPtr_t  cfgValidation (problem().configValidations());
        core::  PathValidationPtr_t pathValidation (problem().pathValidation());
        core::    ValidationReportPtr_t cfgReport;
        core::PathValidationReportPtr_t pathReport;

        core::interval_t timeRange (sm->timeRange());

        std::vector<core::Configuration_t> qs (configurations(*problem().initConfig()));
        if (qs.empty()) {
          hppDout (info, "Failed to generate initial configs.");
          return;
        }

        // Generate a valid initial configuration.
        bool success = false;
        bool resetRightHandSide = true;
        std::size_t i;
        Configuration_t q1;
        for (i = 0; i < qs.size(); ++i)
        {
          if (resetRightHandSide) {
            constraints->rightHandSideAt (timeRange.first);
            resetRightHandSide = false;
          }
          Configuration_t& q (qs[i]);
          if (!constraints->apply (q)) continue;
          if (!cfgValidation->validate (q, cfgReport)) continue;
          resetRightHandSide = true;

          q1 = q;
          success = true;
          for (int j = 1; j <= nDiscreteSteps_; ++j) {
            value_type t = timeRange.first + j * (timeRange.second - timeRange.first) / nDiscreteSteps_;
            constraints->rightHandSideAt (t);
            hppDout (info, "RHS: " << setpyformat << constraints->rightHandSide().transpose());
            if (!constraints->apply (q)) {
              hppDout (info, "Failed to generate destination config.\n" << setpyformat
                  << *constraints
                  << "\nq=" << one_line (q));
              success = false;
              break;
            }
          }
          if (!success) continue;
          success = false;

          if (!cfgValidation->validate (q, cfgReport)) {
            hppDout (info, "Destination config is in collision.");
            continue;
          }

          core::PathPtr_t path = (*sm) (q1, q);
          if (!path) {
            hppDout (info, "Steering method failed.\n" << setpyformat
                << one_line(q1) << '\n'
                << one_line(q ) << '\n'
                );
            continue;
          }

          core::PathPtr_t validPart;
          if (!pathValidation->validate (path, false, validPart, pathReport)) {
            hppDout (info, "Path is in collision.");
            continue;
          }

          roadmap()->initNode (boost::make_shared<Configuration_t>(q1));
          core::NodePtr_t init = roadmap()->   initNode ();
          core::NodePtr_t goal = roadmap()->addGoalNode (boost::make_shared<Configuration_t>(q ));
          roadmap()->addEdge (init, goal, path);
          success = true;
          if (feasibilityOnly_) break;
        }
      }

      std::vector<core::Configuration_t> EndEffectorTrajectory::configurations(const core::Configuration_t& q_init)
      {
        if (!ikSolverInit_) {
          std::vector<core::Configuration_t> configs(nRandomConfig_ + 1);
          configs[0] = q_init;
          for (int i = 1; i < nRandomConfig_ + 1; ++i)
            problem().configurationShooter()->shoot(configs[i]);
          return configs;
        }

        // TODO Compute the target and call ikSolverInit_
        // See https://gepgitlab.laas.fr/airbus-xtct/hpp_airbus_xtct for an
        // example using IKFast.
        throw std::runtime_error ("Using an IkSolverInitialization is not implemented yet");
      }

      EndEffectorTrajectory::EndEffectorTrajectory (const core::Problem& problem)
        : core::PathPlanner (problem)
      {}

      EndEffectorTrajectory::EndEffectorTrajectory (const core::Problem& problem, const core::RoadmapPtr_t& roadmap)
        : core::PathPlanner (problem, roadmap)
      {}

      void EndEffectorTrajectory::checkFeasibilityOnly (bool enable)
      {
        feasibilityOnly_ = enable;
      }

      void EndEffectorTrajectory::init (const EndEffectorTrajectoryWkPtr_t& weak)
      {
        core::PathPlanner::init (weak);
        weak_ = weak;
        nRandomConfig_ = 10;
        nDiscreteSteps_ = 1;
        feasibilityOnly_ = true;
      }
    } // namespace pathPlanner
  } // namespace manipulation
} // namespace hpp
