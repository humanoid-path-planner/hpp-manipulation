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

#include <hpp/core/random-shortcut.hh>
#include <hpp/core/path-optimization/partial-shortcut.hh>
#include <hpp/core/path-projector/progressive.hh>
#include <hpp/core/path-projector/dichotomy.hh>
#include <hpp/core/path-projector/global.hh>
#include <hpp/core/path-projector/recursive-hermite.hh>
#include <hpp/core/roadmap.hh>
#include <hpp/core/steering-method/straight.hh>
#include <hpp/core/comparison-type.hh>

#include "hpp/manipulation/package-config.hh" // HPP_MANIPULATION_HAS_WHOLEBODY_STEP

#include "hpp/manipulation/problem.hh"
#include "hpp/manipulation/device.hh"
#include "hpp/manipulation/handle.hh"
#include "hpp/manipulation/graph/graph.hh"
#include "hpp/manipulation/symbolic-planner.hh"
#include "hpp/manipulation/manipulation-planner.hh"
#include "hpp/manipulation/roadmap.hh"
#include "hpp/manipulation/constraint-set.hh"
#include "hpp/manipulation/graph-optimizer.hh"
#include "hpp/manipulation/graph-path-validation.hh"
#include "hpp/manipulation/graph-node-optimizer.hh"
#include "hpp/manipulation/graph-steering-method.hh"
#include "hpp/manipulation/path-optimization/config-optimization.hh"
#include "hpp/manipulation/path-optimization/keypoints.hh"
#include "hpp/manipulation/path-optimization/spline-gradient-based.hh"
#include "hpp/manipulation/problem-target/state.hh"

#if HPP_MANIPULATION_HAS_WHOLEBODY_STEP
#include <hpp/wholebody-step/small-steps.hh>
#include "hpp/manipulation/path-optimization/small-steps.hh"
#endif

namespace hpp {
  namespace manipulation {
    namespace {
      struct PartialShortcutTraits :
        core::pathOptimization::PartialShortcutTraits {
          static bool removeLockedJoints () { return false; }
      };

      template <typename InnerConfigOptimizationTraits>
        struct GraphConfigOptimizationTraits {
          static core::PathOptimizerPtr_t create (const core::Problem& problem)
          {
            return core::pathOptimization::ConfigOptimization::
              createWithTraits <InnerConfigOptimizationTraits> (problem);
          }
        };

      template <typename PathProjectorType>
      core::PathProjectorPtr_t createPathProjector
      (const core::Problem& problem, const value_type& step)
      {
        GraphSteeringMethodPtr_t gsm =
          HPP_DYNAMIC_PTR_CAST (GraphSteeringMethod, problem.steeringMethod());
        if (!gsm) throw std::logic_error ("The steering method should be of type"
            " GraphSteeringMethod");
        return PathProjectorType::create (problem.distance(),
            gsm->innerSteeringMethod(), step);
      }
    }

    std::ostream& operator<< (std::ostream& os, const Device& robot)
    {
      return robot.print (os);
    }

    ProblemSolver::ProblemSolver () :
      core::ProblemSolver (), robot_ (), problem_ (0x0)
    {
      parent_t::add<core::PathPlannerBuilder_t>
        ("M-RRT", ManipulationPlanner::create);
      parent_t::add<core::PathPlannerBuilder_t>
        ("SymbolicPlanner", SymbolicPlanner::create);
      using core::PathOptimizerBuilder_t;
      parent_t::add <PathOptimizerBuilder_t> ("Graph-RandomShortcut",
          GraphOptimizer::create <core::RandomShortcut>);
      parent_t::add <PathOptimizerBuilder_t> ("PartialShortcut", core::pathOptimization::
          PartialShortcut::createWithTraits <PartialShortcutTraits>);
      parent_t::add <PathOptimizerBuilder_t> ("Graph-PartialShortcut",
          GraphOptimizer::create <core::pathOptimization::PartialShortcut>);
      parent_t::add <PathOptimizerBuilder_t> ("ConfigOptimization",
          core::pathOptimization::ConfigOptimization::createWithTraits
          <pathOptimization::ConfigOptimizationTraits>);
      parent_t::add <PathOptimizerBuilder_t> ("Graph-ConfigOptimization",
          GraphOptimizer::create <
          GraphConfigOptimizationTraits
            <pathOptimization::ConfigOptimizationTraits>
            >);

      parent_t::add <core::PathProjectorBuilder_t> ("Progressive",
          createPathProjector <core::pathProjector::Progressive>);
      parent_t::add <core::PathProjectorBuilder_t> ("Dichotomy",
          createPathProjector <core::pathProjector::Dichotomy>);
      parent_t::add <core::PathProjectorBuilder_t> ("Global",
          createPathProjector <core::pathProjector::Global>);
      parent_t::add <core::PathProjectorBuilder_t> ("RecursiveHermite",
          createPathProjector <core::pathProjector::RecursiveHermite>);

      // parent_t::add <PathOptimizerBuilder_t> ("SplineGradientBased_cannonical1",pathOptimization::SplineGradientBased<core::path::CanonicalPolynomeBasis, 1>::createFromCore);
      // parent_t::add <PathOptimizerBuilder_t> ("SplineGradientBased_cannonical2",pathOptimization::SplineGradientBased<core::path::CanonicalPolynomeBasis, 2>::createFromCore);
      // parent_t::add <PathOptimizerBuilder_t> ("SplineGradientBased_cannonical3",pathOptimization::SplineGradientBased<core::path::CanonicalPolynomeBasis, 3>::createFromCore);
      parent_t::add <PathOptimizerBuilder_t> ("SplineGradientBased_bezier1",pathOptimization::SplineGradientBased<core::path::BernsteinBasis, 1>::createFromCore);
      // parent_t::add <PathOptimizerBuilder_t> ("SplineGradientBased_bezier2",pathOptimization::SplineGradientBased<core::path::BernsteinBasis, 2>::createFromCore);
      parent_t::add <PathOptimizerBuilder_t> ("SplineGradientBased_bezier3",pathOptimization::SplineGradientBased<core::path::BernsteinBasis, 3>::createFromCore);

      using core::SteeringMethodBuilder_t;
      parent_t::add <SteeringMethodBuilder_t> ("Graph-SteeringMethodStraight",
          GraphSteeringMethod::create <core::SteeringMethodStraight>);

      parent_t::add <PathOptimizerBuilder_t> ("KeypointsShortcut",
          pathOptimization::Keypoints::create);

#if HPP_MANIPULATION_HAS_WHOLEBODY_STEP
      parent_t::add <PathOptimizerBuilder_t>
        ("Walkgen", wholebodyStep::SmallSteps::create);
      parent_t::add <PathOptimizerBuilder_t>
        ("Graph-Walkgen", pathOptimization::SmallSteps::create);
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
      if (problem_)
        delete (problem_);
      initializeProblem (new Problem (robot_));
    }

    void ProblemSolver::initializeProblem (ProblemPtr_t problem)
    {
      problem_ = problem;
      core::ProblemSolver::initializeProblem (problem_);
      if (problem_->pathValidation ())
        problem_->pathValidation ()->constraintGraph (constraintGraph_);
    }

    void ProblemSolver::constraintGraph (const graph::GraphPtr_t& graph)
    {
      constraintGraph_ = graph;
      RoadmapPtr_t r = HPP_DYNAMIC_PTR_CAST (Roadmap, roadmap());
      if (r) r->constraintGraph (graph);
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
    }

    void ProblemSolver::createPlacementConstraint
    (const std::string& name, const StringList_t& surface1,
     const StringList_t& surface2, const value_type& margin)
    {
      if (!robot_) throw std::runtime_error ("No robot loaded");
      using constraints::ConvexShape;
      using constraints::ConvexShapeContactPtr_t;
      using constraints::ConvexShapeContactComplement;
      using constraints::ConvexShapeContactComplementPtr_t;
      std::string complementName (name + "/complement");
      std::pair < ConvexShapeContactPtr_t,
		  ConvexShapeContactComplementPtr_t > constraints
	(ConvexShapeContactComplement::createPair
	 (name, complementName, robot_));

      JointAndShapes_t l;
      for (StringList_t::const_iterator it1 = surface1.begin ();
          it1 != surface1.end(); ++it1) {
        if (!robot_->has <JointAndShapes_t> (*it1))
          throw std::runtime_error ("First list of triangles not found.");
        l = robot_->get <JointAndShapes_t> (*it1);
        for (JointAndShapes_t::const_iterator it = l.begin ();
            it != l.end(); ++it) {
          constraints.first->addObject (ConvexShape (it->second, it->first));
        }
      }

      for (StringList_t::const_iterator it2 = surface2.begin ();
          it2 != surface2.end(); ++it2) {
        // Search first robot triangles
        if (robot_->has <JointAndShapes_t> (*it2))
          l = robot_->get <JointAndShapes_t> (*it2);
        // and then environment triangles.
        else if (ThisC_t::has <JointAndShapes_t> (*it2))
          l = ThisC_t::get <JointAndShapes_t> (*it2);
        else throw std::runtime_error ("Second list of triangles not found.");
        for (JointAndShapes_t::const_iterator it = l.begin ();
            it != l.end(); ++it) {
          constraints.first->addFloor (ConvexShape (it->second, it->first));
        }
      }

      constraints.first->setNormalMargin (margin);

      addNumericalConstraint (name, NumericalConstraint::create
			      (constraints.first));
      addNumericalConstraint (complementName, NumericalConstraint::create
			      (constraints.second, core::Equality::create ()));
    }

    void ProblemSolver::createPrePlacementConstraint
    (const std::string& name, const StringList_t& surface1,
     const StringList_t& surface2, const value_type& width,
     const value_type& margin)
    {
      if (!robot_) throw std::runtime_error ("No robot loaded");
      using constraints::ConvexShape;
      using constraints::ConvexShapeContact;
      using constraints::ConvexShapeContactPtr_t;

      ConvexShapeContactPtr_t cvxShape = ConvexShapeContact::create (name, robot_);

      JointAndShapes_t l;
      for (StringList_t::const_iterator it1 = surface1.begin ();
          it1 != surface1.end(); ++it1) {
        if (!robot_->has <JointAndShapes_t> (*it1))
          throw std::runtime_error ("First list of triangles not found.");
        l = robot_->get <JointAndShapes_t> (*it1);

        for (JointAndShapes_t::const_iterator it = l.begin ();
            it != l.end(); ++it) {
          cvxShape->addObject (ConvexShape (it->second, it->first));
        }
      }

      for (StringList_t::const_iterator it2 = surface2.begin ();
          it2 != surface2.end(); ++it2) {
        // Search first robot triangles
        if (robot_->has <JointAndShapes_t> (*it2))
          l = robot_->get <JointAndShapes_t> (*it2);
        // and then environment triangles.
        else if (ThisC_t::has <JointAndShapes_t> (*it2))
          l = ThisC_t::get <JointAndShapes_t> (*it2);
        else throw std::runtime_error ("Second list of triangles not found.");

        for (JointAndShapes_t::const_iterator it = l.begin ();
            it != l.end(); ++it) {
          cvxShape->addFloor (ConvexShape (it->second, it->first));
        }
      }

      cvxShape->setNormalMargin (margin + width);

      addNumericalConstraint (name, NumericalConstraint::create (cvxShape));
    }

    void ProblemSolver::createGraspConstraint
    (const std::string& name, const std::string& gripper,
     const std::string& handle)
    {
      GripperPtr_t g = robot_->get <GripperPtr_t> (gripper);
      if (!g) throw std::runtime_error ("No gripper with name " + gripper + ".");
      HandlePtr_t h = robot_->get <HandlePtr_t> (handle);
      if (!h) throw std::runtime_error ("No handle with name " + handle + ".");
      const std::string cname = name + "/complement";
      NumericalConstraintPtr_t constraint (h->createGrasp (g, name));
      NumericalConstraintPtr_t complement (h->createGraspComplement (g, cname));
      addNumericalConstraint ( name, constraint);
      addNumericalConstraint (cname, complement);
    }

    void ProblemSolver::createPreGraspConstraint
    (const std::string& name, const std::string& gripper,
     const std::string& handle)
    {
      GripperPtr_t g = robot_->get <GripperPtr_t> (gripper);
      if (!g) throw std::runtime_error ("No gripper with name " + gripper + ".");
      HandlePtr_t h = robot_->get <HandlePtr_t> (handle);
      if (!h) throw std::runtime_error ("No handle with name " + handle + ".");

      value_type c = h->clearance () + g->clearance ();
      NumericalConstraintPtr_t constraint = h->createPreGrasp (g, c, name);
      addNumericalConstraint (name, constraint);
    }

    void ProblemSolver::pathValidationType (const std::string& type,
        const value_type& tolerance)
    {
      parent_t::pathValidationType(type, tolerance);
      problem_->setPathValidationFactory (
          parent_t::get<core::PathValidationBuilder_t>(type),
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
      problemTarget::StatePtr_t t =  problemTarget::State::create(NULL);
      t->target(state);
      target_ = t;
    }
  } // namespace manipulation
} // namespace hpp
