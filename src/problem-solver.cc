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
#include "hpp/manipulation/symbolic-planner.hh"
#include "hpp/manipulation/manipulation-planner.hh"
#include "hpp/manipulation/roadmap.hh"
#include "hpp/manipulation/constraint-set.hh"
#include "hpp/manipulation/graph-optimizer.hh"
#include "hpp/manipulation/graph-path-validation.hh"
#include "hpp/manipulation/graph-node-optimizer.hh"
#include "hpp/manipulation/path-optimization/config-optimization.hh"
#include "hpp/manipulation/path-optimization/keypoints.hh"
#include "hpp/manipulation/path-optimization/spline-gradient-based.hh"
#include "hpp/manipulation/problem-target/state.hh"
#include "hpp/manipulation/steering-method/cross-state-optimization.hh"
#include "hpp/manipulation/steering-method/graph.hh"

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

      template <typename InnerConfigOptimizationTraits>
        struct GraphConfigOptimizationTraits {
          static core::PathOptimizerPtr_t create (const core::Problem& problem)
          {
            return core::pathOptimization::ConfigOptimization::
              createWithTraits <InnerConfigOptimizationTraits> (problem);
          }
        };

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
      core::ProblemSolver (), robot_ (), problem_ (0x0)
    {
      robots.add ("hpp::manipulation::Device", manipulation::Device::create);
      robotType ("hpp::manipulation::Device");

      pathPlanners.add ("M-RRT", ManipulationPlanner::create);
      pathPlanners.add ("SymbolicPlanner", SymbolicPlanner::create);

      pathOptimizers.add ("Graph-RandomShortcut",
          GraphOptimizer::create <core::RandomShortcut>);
      pathOptimizers.add ("PartialShortcut", core::pathOptimization::
          PartialShortcut::createWithTraits <PartialShortcutTraits>);
      pathOptimizers.add ("Graph-PartialShortcut",
          GraphOptimizer::create <core::pathOptimization::PartialShortcut>);
      pathOptimizers.add ("ConfigOptimization",
          core::pathOptimization::ConfigOptimization::createWithTraits
          <pathOptimization::ConfigOptimizationTraits>);
      pathOptimizers.add ("Graph-ConfigOptimization",
          GraphOptimizer::create <
          GraphConfigOptimizationTraits
            <pathOptimization::ConfigOptimizationTraits>
            >);

      pathProjectors.add ("Progressive",
          createPathProjector <core::pathProjector::Progressive>);
      pathProjectors.add ("Dichotomy",
          createPathProjector <core::pathProjector::Dichotomy>);
      pathProjectors.add ("Global",
          createPathProjector <core::pathProjector::Global>);
      pathProjectors.add ("RecursiveHermite",
          createPathProjector <core::pathProjector::RecursiveHermite>);

      // pathOptimizers.add ("SplineGradientBased_cannonical1",pathOptimization::SplineGradientBased<core::path::CanonicalPolynomeBasis, 1>::createFromCore);
      // pathOptimizers.add ("SplineGradientBased_cannonical2",pathOptimization::SplineGradientBased<core::path::CanonicalPolynomeBasis, 2>::createFromCore);
      // pathOptimizers.add ("SplineGradientBased_cannonical3",pathOptimization::SplineGradientBased<core::path::CanonicalPolynomeBasis, 3>::createFromCore);
      pathOptimizers.add ("SplineGradientBased_bezier1",pathOptimization::SplineGradientBased<core::path::BernsteinBasis, 1>::createFromCore);
      // pathOptimizers.add ("SplineGradientBased_bezier2",pathOptimization::SplineGradientBased<core::path::BernsteinBasis, 2>::createFromCore);
      pathOptimizers.add ("SplineGradientBased_bezier3",pathOptimization::SplineGradientBased<core::path::BernsteinBasis, 3>::createFromCore);

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

      pathOptimizers.add ("KeypointsShortcut",
          pathOptimization::Keypoints::create);

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
      if (problem_)
        delete (problem_);
      initializeProblem (new Problem (robot_));
    }

    void ProblemSolver::initializeProblem (ProblemPtr_t problem)
    {
      problem_ = problem;
      core::ProblemSolver::initializeProblem (problem_);
      if (constraintGraph_) {
        problem_->constraintGraph (constraintGraph_);
        if (problem_->pathValidation ())
          problem_->pathValidation ()->constraintGraph (constraintGraph_);
      }
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
        if (!robot_->jointAndShapes.has (*it1))
          throw std::runtime_error ("First list of triangles not found.");
        l = robot_->jointAndShapes.get (*it1);
        for (JointAndShapes_t::const_iterator it = l.begin ();
            it != l.end(); ++it) {
          constraints.first->addObject (ConvexShape (it->second, it->first));
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
        for (JointAndShapes_t::const_iterator it = l.begin ();
            it != l.end(); ++it) {
          constraints.first->addFloor (ConvexShape (it->second, it->first));
        }
      }

      constraints.first->setNormalMargin (margin);

      addNumericalConstraint (name, Implicit::create
			      (constraints.first));
      addNumericalConstraint (complementName, Implicit::create
			      (constraints.second,
                               ComparisonTypes_t
                               (constraints.second->outputSize(),
                                constraints::Equality))
                              );
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
        if (!robot_->jointAndShapes.has (*it1))
          throw std::runtime_error ("First list of triangles not found.");
        l = robot_->jointAndShapes.get (*it1);

        for (JointAndShapes_t::const_iterator it = l.begin ();
            it != l.end(); ++it) {
          cvxShape->addObject (ConvexShape (it->second, it->first));
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

        for (JointAndShapes_t::const_iterator it = l.begin ();
            it != l.end(); ++it) {
          cvxShape->addFloor (ConvexShape (it->second, it->first));
        }
      }

      cvxShape->setNormalMargin (margin + width);

      addNumericalConstraint (name, Implicit::create (cvxShape));
    }

    void ProblemSolver::createGraspConstraint
    (const std::string& name, const std::string& gripper,
     const std::string& handle)
    {
      if (!constraintGraph ()) {
        throw std::runtime_error ("The graph is not defined.");
      }
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
      assert (problem_);
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
      problemTarget::StatePtr_t t =  problemTarget::State::create(NULL);
      t->target(state);
      target_ = t;
    }
  } // namespace manipulation
} // namespace hpp
