// Copyright (c) 2014 CNRS
// Author: Florent Lamiraux
//
// This file is part of hpp-manipulation-corba.
// hpp-manipulation-corba is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-manipulation-corba is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-manipulation-corba.  If not, see
// <http://www.gnu.org/licenses/>.

#include "hpp/manipulation/problem-solver.hh"

#include <boost/bind.hpp>

#include <hpp/util/pointer.hh>
#include <hpp/util/debug.hh>

#include <hpp/pinocchio/gripper.hh>

#include <hpp/constraints/convex-shape-contact.hh>
#include <hpp/constraints/explicit/convex-shape-contact.hh>

#include <hpp/core/path-optimization/random-shortcut.hh>
#include <hpp/core/path-optimization/partial-shortcut.hh>
#include <hpp/core/path-projector/progressive.hh>
#include <hpp/core/path-projector/dichotomy.hh>
#include <hpp/core/path-projector/global.hh>
#include <hpp/core/path-projector/recursive-hermite.hh>
#include <hpp/core/path-validation/discretized-collision-checking.hh>
#include <hpp/core/path-validation/discretized-joint-bound.hh>
#include <hpp/core/continuous-validation/dichotomy.hh>
#include <hpp/core/continuous-validation/progressive.hh>
#include <hpp/core/roadmap.hh>
#include <hpp/core/steering-method/dubins.hh>
#include <hpp/core/steering-method/hermite.hh>
#include <hpp/core/steering-method/reeds-shepp.hh>
#include <hpp/core/steering-method/snibud.hh>
#include <hpp/core/steering-method/straight.hh>

#include "hpp/manipulation/package-config.hh" // HPP_MANIPULATION_HAS_WHOLEBODY_STEP

#include "hpp/manipulation/problem.hh"
#include "hpp/manipulation/device.hh"
#include "hpp/manipulation/handle.hh"
#include "hpp/manipulation/graph/graph.hh"
#include "hpp/manipulation/manipulation-planner.hh"
#include "hpp/manipulation/roadmap.hh"
#include "hpp/manipulation/constraint-set.hh"
#include "hpp/manipulation/graph-optimizer.hh"
#include "hpp/manipulation/graph-path-validation.hh"
#include "hpp/manipulation/graph-node-optimizer.hh"
#include "hpp/manipulation/path-optimization/random-shortcut.hh"
#include "hpp/manipulation/path-optimization/enforce-transition-semantic.hh"
#include "hpp/manipulation/path-planner/end-effector-trajectory.hh"
#include "hpp/manipulation/problem-target/state.hh"
#include "hpp/manipulation/steering-method/cross-state-optimization.hh"
#include "hpp/manipulation/steering-method/graph.hh"
#include "hpp/manipulation/steering-method/end-effector-trajectory.hh"

#if HPP_MANIPULATION_HAS_WHOLEBODY_STEP
#include <hpp/wholebody-step/small-steps.hh>
#include "hpp/manipulation/path-optimization/small-steps.hh"
#endif

namespace hpp {
  namespace manipulation {
    typedef constraints::Implicit Implicit;
    typedef constraints::ImplicitPtr_t ImplicitPtr_t;
    namespace {
      struct PartialShortcutTraits :
        core::pathOptimization::PartialShortcutTraits {
          static bool removeLockedJoints () { return false; }
      };

#define MAKE_GRAPH_PATH_VALIDATION_BUILDER(name, function)                     \
      PathValidationPtr_t create ## name ## GraphPathValidation (              \
          const core::DevicePtr_t& robot, const value_type& stepSize)          \
      {                                                                        \
        return GraphPathValidation::create (function (robot, stepSize));       \
      }
      MAKE_GRAPH_PATH_VALIDATION_BUILDER(DiscretizedCollision             , core::pathValidation::createDiscretizedCollisionChecking)
      MAKE_GRAPH_PATH_VALIDATION_BUILDER(DiscretizedJointBound            , core::pathValidation::createDiscretizedJointBound)
      //MAKE_GRAPH_PATH_VALIDATION_BUILDER(DiscretizedCollisionAndJointBound, createDiscretizedJointBoundAndCollisionChecking)

      template <typename ParentSM_t, typename ChildSM_t>
      core::SteeringMethodPtr_t createSMWithGuess
      (const core::Problem& problem)
      {
        boost::shared_ptr<ParentSM_t> sm = ParentSM_t::create (problem);
        sm->innerSteeringMethod (ChildSM_t::createWithGuess (problem));
        return sm;
      }

      template <typename PathProjectorType>
      core::PathProjectorPtr_t createPathProjector
      (const core::Problem& problem, const value_type& step)
      {
        steeringMethod::GraphPtr_t gsm =
          HPP_DYNAMIC_PTR_CAST (steeringMethod::Graph, problem.steeringMethod());
        if (!gsm) throw std::logic_error ("The steering method should be of type"
            " steeringMethod::Graph");
        return PathProjectorType::create (problem.distance(),
            gsm->innerSteeringMethod(), step);
      }
    }

    std::ostream& operator<< (std::ostream& os, const Device& robot)
    {
      return robot.print (os);
    }

    ProblemSolver::ProblemSolver () :
      core::ProblemSolver (), robot_ (), problem_ ()
    {
      robots.add ("hpp::manipulation::Device", manipulation::Device::create);
      robotType ("hpp::manipulation::Device");

      pathPlanners.add ("M-RRT", ManipulationPlanner::create);
      pathPlanners.add ("EndEffectorTrajectory", pathPlanner::EndEffectorTrajectory::createWithRoadmap);

      pathValidations.add ("Graph-Discretized"                      , createDiscretizedCollisionGraphPathValidation);
      pathValidations.add ("Graph-DiscretizedCollision"             , createDiscretizedCollisionGraphPathValidation);
      pathValidations.add ("Graph-DiscretizedJointBound"            , createDiscretizedJointBoundGraphPathValidation);
      //pathValidations.add ("Graph-DiscretizedCollisionAndJointBound", createDiscretizedCollisionAndJointBoundGraphPathValidation);
      pathValidations.add ("Graph-Dichotomy"  , GraphPathValidation::create<core::continuousValidation::Dichotomy  >);
      pathValidations.add ("Graph-Progressive", GraphPathValidation::create<core::continuousValidation::Progressive>);

      // TODO Uncomment to make Graph-Discretized the default.
      //pathValidationType ("Graph-Discretized", 0.05);

      pathOptimizers.add ("RandomShortcut",
          pathOptimization::RandomShortcut::create);
      pathOptimizers.add ("Graph-RandomShortcut",
          GraphOptimizer::create <core::pathOptimization::RandomShortcut>);
      pathOptimizers.add ("PartialShortcut", core::pathOptimization::
          PartialShortcut::createWithTraits <PartialShortcutTraits>);
      pathOptimizers.add ("Graph-PartialShortcut",
          GraphOptimizer::create <core::pathOptimization::PartialShortcut>);
      pathOptimizers.add ("EnforceTransitionSemantic",
          pathOptimization::EnforceTransitionSemantic::create);

      pathProjectors.add ("Progressive",
          createPathProjector <core::pathProjector::Progressive>);
      pathProjectors.add ("Dichotomy",
          createPathProjector <core::pathProjector::Dichotomy>);
      pathProjectors.add ("Global",
          createPathProjector <core::pathProjector::Global>);
      pathProjectors.add ("RecursiveHermite",
          createPathProjector <core::pathProjector::RecursiveHermite>);

      steeringMethods.add ("Graph-SteeringMethodStraight",
          steeringMethod::Graph::create <core::SteeringMethodStraight>);
      steeringMethods.add ("Graph-Straight",
          steeringMethod::Graph::create <core::steeringMethod::Straight>);
      steeringMethods.add ("Graph-Hermite",
          steeringMethod::Graph::create <core::steeringMethod::Hermite>);
      steeringMethods.add ("Graph-ReedsShepp",
          createSMWithGuess <steeringMethod::Graph, core::steeringMethod::ReedsShepp>);
      steeringMethods.add ("Graph-Dubins",
          createSMWithGuess <steeringMethod::Graph, core::steeringMethod::Dubins>);
      steeringMethods.add ("Graph-Snibud",
          createSMWithGuess <steeringMethod::Graph, core::steeringMethod::Snibud>);
      steeringMethods.add ("CrossStateOptimization-Straight",
          steeringMethod::CrossStateOptimization::create<core::steeringMethod::Straight>);
      steeringMethods.add ("CrossStateOptimization-ReedsShepp",
          createSMWithGuess <steeringMethod::CrossStateOptimization, core::steeringMethod::ReedsShepp>);
      steeringMethods.add ("CrossStateOptimization-Dubins",
          createSMWithGuess <steeringMethod::CrossStateOptimization, core::steeringMethod::Dubins>);
      steeringMethods.add ("CrossStateOptimization-Snibud",
          createSMWithGuess <steeringMethod::CrossStateOptimization, core::steeringMethod::Snibud>);
      steeringMethods.add ("EndEffectorTrajectory", steeringMethod::EndEffectorTrajectory::create);

#if HPP_MANIPULATION_HAS_WHOLEBODY_STEP
      pathOptimizers.add ("Walkgen", wholebodyStep::SmallSteps::create);
      pathOptimizers.add ("Graph-Walkgen", pathOptimization::SmallSteps::create);
#endif

      pathPlannerType ("M-RRT");
      steeringMethodType ("Graph-SteeringMethodStraight");
    }

    ProblemSolverPtr_t ProblemSolver::create ()
    {
      return ProblemSolverPtr_t (new ProblemSolver ());
    }

    void ProblemSolver::resetProblem ()
    {
      ProblemPtr_t p (Problem::create(robot_));
      if (problem_) {
        p->parameters = problem_->parameters;
      }
      initializeProblem (p);
    }

    void ProblemSolver::initializeProblem (ProblemPtr_t problem)
    {
      problem_ = problem;
      core::ProblemSolver::initializeProblem (problem_);
      if (constraintGraph_)
        problem_->constraintGraph (constraintGraph_);
      value_type tolerance;
      const std::string& type = parent_t::pathValidationType (tolerance);
      problem_->setPathValidationFactory (pathValidations.get(type), tolerance);
    }

    void ProblemSolver::constraintGraph (const std::string& graphName)
    {
      if (!graphs.has (graphName))
        throw std::invalid_argument ("ProblemSolver has no graph named " + graphName);
      constraintGraph_ = graphs.get(graphName);
      RoadmapPtr_t r = HPP_DYNAMIC_PTR_CAST (Roadmap, roadmap());
      if (r) r->constraintGraph (constraintGraph_);
      if (problem_) problem_->constraintGraph (constraintGraph_);
    }

    graph::GraphPtr_t ProblemSolver::constraintGraph () const
    {
      return constraintGraph_;
    }

    void ProblemSolver::initConstraintGraph ()
    {
      if (!constraintGraph_)
        throw std::runtime_error ("The graph is not defined.");
      initSteeringMethod();
      constraintGraph_->initialize();

      for (std::size_t i = 0; i < constraintsAndComplements.size(); ++i) {
        const ConstraintAndComplement_t& c = constraintsAndComplements[i];
        constraintGraph ()->registerConstraints (c.constraint, c.complement, c.both);
      }
    }

    void ProblemSolver::createPlacementConstraint
    (const std::string& name, const StringList_t& surface1,
     const StringList_t& surface2, const value_type& margin)
    {
      if (!robot_) throw std::runtime_error ("No robot loaded");
      JointAndShapes_t floorSurfaces, objectSurfaces, l;
      for (StringList_t::const_iterator it1 = surface1.begin ();
          it1 != surface1.end(); ++it1) {
        if (!robot_->jointAndShapes.has (*it1))
          throw std::runtime_error ("First list of triangles not found.");
        l = robot_->jointAndShapes.get (*it1);
        for (JointAndShapes_t::const_iterator it(l.begin()); it!=l.end();++it)
        {
          objectSurfaces.push_back(*it);
        }
      }

      for (StringList_t::const_iterator it2 = surface2.begin ();
          it2 != surface2.end(); ++it2) {
        // Search first robot triangles
        if (robot_->jointAndShapes.has (*it2))
          l = robot_->jointAndShapes.get (*it2);
        // and then environment triangles.
        else if (jointAndShapes.has (*it2))
          l = jointAndShapes.get (*it2);
        else throw std::runtime_error ("Second list of triangles not found.");
        for (JointAndShapes_t::const_iterator it(l.begin()); it!=l.end();++it)
        {
          floorSurfaces.push_back(*it);
        }
      }

      typedef hpp::constraints::explicit_::ConvexShapeContact Constraint_t;
      Constraint_t::Constraints_t constraints
        (Constraint_t::createConstraintAndComplement
         (name, robot_, floorSurfaces, objectSurfaces, margin));

      addNumericalConstraint(std::get<0>(constraints)->function().name(),
                             std::get<0>(constraints));
      addNumericalConstraint(std::get<1>(constraints)->function().name(),
                             std::get<1>(constraints));
      addNumericalConstraint(std::get<2>(constraints)->function().name(),
                             std::get<2>(constraints));
      // Set security margin to contact constraint
      assert(HPP_DYNAMIC_PTR_CAST(constraints::ConvexShapeContact,
                                  std::get<0>(constraints)->functionPtr()));
      constraints::ConvexShapeContactPtr_t contactFunction
        (HPP_STATIC_PTR_CAST(constraints::ConvexShapeContact,
                             std::get<0>(constraints)->functionPtr()));
      contactFunction->setNormalMargin(margin);
      constraintsAndComplements.push_back (
          ConstraintAndComplement_t (std::get<0>(constraints),
                                     std::get<1>(constraints),
                                     std::get<2>(constraints)));
      if (constraintGraph ())
        constraintGraph ()->registerConstraints(std::get<0>(constraints),
                                                std::get<1>(constraints),
                                                std::get<2>(constraints));
    }

    void ProblemSolver::createPrePlacementConstraint
    (const std::string& name, const StringList_t& surface1,
     const StringList_t& surface2, const value_type& width,
     const value_type& margin)
    {
      if (!robot_) throw std::runtime_error ("No robot loaded");
      JointAndShapes_t floorSurfaces, objectSurfaces, l;
      for (StringList_t::const_iterator it1 = surface1.begin ();
          it1 != surface1.end(); ++it1) {
        if (!robot_->jointAndShapes.has (*it1))
          throw std::runtime_error ("First list of triangles not found.");
        l = robot_->jointAndShapes.get (*it1);
        for (JointAndShapes_t::const_iterator it(l.begin()); it!=l.end();++it)
        {
          objectSurfaces.push_back(*it);
        }
      }

      for (StringList_t::const_iterator it2 = surface2.begin ();
          it2 != surface2.end(); ++it2) {
        // Search first robot triangles
        if (robot_->jointAndShapes.has (*it2))
          l = robot_->jointAndShapes.get (*it2);
        // and then environment triangles.
        else if (jointAndShapes.has (*it2))
          l = jointAndShapes.get (*it2);
        else throw std::runtime_error ("Second list of triangles not found.");
        for (JointAndShapes_t::const_iterator it(l.begin()); it!=l.end();++it)
        {
          floorSurfaces.push_back(*it);
        }
      }

      hpp::constraints::ConvexShapeContactPtr_t cvxShape
        (hpp::constraints::ConvexShapeContact::create
         (name, robot_, floorSurfaces, objectSurfaces));
      cvxShape->setNormalMargin(margin + width);
      addNumericalConstraint (name, Implicit::create (cvxShape));
    }

    void ProblemSolver::createGraspConstraint
    (const std::string& name, const std::string& gripper,
     const std::string& handle)
    {
      GripperPtr_t g = robot_->grippers.get (gripper, GripperPtr_t());
      if (!g) throw std::runtime_error ("No gripper with name " + gripper + ".");
      HandlePtr_t h = robot_->handles.get (handle, HandlePtr_t());
      if (!h) throw std::runtime_error ("No handle with name " + handle + ".");
      const std::string cname = name + "/complement";
      const std::string bname = name + "/hold";
      ImplicitPtr_t constraint (h->createGrasp (g, name));
      ImplicitPtr_t complement (h->createGraspComplement (g, cname));
      ImplicitPtr_t both (h->createGraspAndComplement (g, bname));
      addNumericalConstraint ( name, constraint);
      addNumericalConstraint (cname, complement);
      addNumericalConstraint (bname, both);

      constraintsAndComplements.push_back (
          ConstraintAndComplement_t (constraint, complement, both));
      if (constraintGraph ())
        constraintGraph ()->registerConstraints (constraint, complement, both);
    }

    void ProblemSolver::createPreGraspConstraint
    (const std::string& name, const std::string& gripper,
     const std::string& handle)
    {
      GripperPtr_t g = robot_->grippers.get (gripper, GripperPtr_t());
      if (!g) throw std::runtime_error ("No gripper with name " + gripper + ".");
      HandlePtr_t h = robot_->handles.get (handle, HandlePtr_t());
      if (!h) throw std::runtime_error ("No handle with name " + handle + ".");

      value_type c = h->clearance () + g->clearance ();
      ImplicitPtr_t constraint = h->createPreGrasp (g, c, name);
      addNumericalConstraint (name, constraint);
    }

    void ProblemSolver::pathValidationType (const std::string& type,
        const value_type& tolerance)
    {
      parent_t::pathValidationType(type, tolerance);
      if (problem_)
        problem_->setPathValidationFactory (
            pathValidations.get(type),
            tolerance);
    }

    void ProblemSolver::resetRoadmap ()
    {
      if (!problem ())
        throw std::runtime_error ("The problem is not defined.");
      if (roadmap())
        roadmap()->clear();
      RoadmapPtr_t r (Roadmap::create (problem ()->distance (), problem ()->robot ()));
      if (constraintGraph_) r->constraintGraph (constraintGraph_);
      roadmap (r);
    }

    void ProblemSolver::setTargetState (const graph::StatePtr_t state)
    {
      problemTarget::StatePtr_t t =  problemTarget::State::create(ProblemPtr_t());
      t->target(state);
      target_ = t;
    }
  } // namespace manipulation
} // namespace hpp
