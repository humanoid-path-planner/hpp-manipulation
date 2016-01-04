//
// Copyright (c) 2014 CNRS
// Authors: Florent Lamiraux, Joseph Mirabel
//
//
// This file is part of hpp-manipulation.
// hpp-manipulation is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-manipulation is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-manipulation. If not, see
// <http://www.gnu.org/licenses/>.

#ifndef HPP_MANIPULATION_FWD_HH
# define HPP_MANIPULATION_FWD_HH

# include <map>
# include <hpp/core/fwd.hh>
# include <hpp/fcl/shape/geometric_shapes.h>

# include <hpp/manipulation/deprecated.hh>

namespace hpp {
  namespace manipulation {
    HPP_PREDEF_CLASS (Device);
    typedef boost::shared_ptr <Device> DevicePtr_t;
    typedef boost::shared_ptr <const Device> DeviceConstPtr_t;
    typedef model::Joint Joint;
    typedef model::JointPtr_t JointPtr_t;
    typedef model::JointConstPtr_t JointConstPtr_t;
    typedef model::Transform3f Transform3f;
    typedef model::Configuration_t Configuration_t;
    typedef model::ConfigurationIn_t ConfigurationIn_t;
    typedef model::ConfigurationOut_t ConfigurationOut_t;
    typedef boost::shared_ptr < model::Configuration_t > ConfigurationPtr_t;
    typedef model::GripperPtr_t GripperPtr_t;
    HPP_PREDEF_CLASS (AxialHandle);
    typedef boost::shared_ptr <AxialHandle> AxialHandlePtr_t;
    HPP_PREDEF_CLASS (Handle);
    typedef boost::shared_ptr <Handle> HandlePtr_t;
    HPP_PREDEF_CLASS (Object);
    typedef boost::shared_ptr <Object> ObjectPtr_t;
    typedef boost::shared_ptr <const Object> ObjectConstPtr_t;
    HPP_PREDEF_CLASS (ProblemSolver);
    typedef ProblemSolver* ProblemSolverPtr_t;
    HPP_PREDEF_CLASS (Problem);
    typedef Problem* ProblemPtr_t;
    HPP_PREDEF_CLASS (Roadmap);
    typedef boost::shared_ptr <Roadmap> RoadmapPtr_t;
    HPP_PREDEF_CLASS (RoadmapNode);
    typedef RoadmapNode* RoadmapNodePtr_t;
    HPP_PREDEF_CLASS (ConnectedComponent);
    typedef boost::shared_ptr<ConnectedComponent> ConnectedComponentPtr_t; 
    HPP_PREDEF_CLASS (WeighedDistance);
    typedef boost::shared_ptr<WeighedDistance> WeighedDistancePtr_t;
    typedef constraints::RelativeOrientation RelativeOrientation;
    typedef constraints::RelativePosition RelativePosition;
    typedef constraints::RelativeOrientationPtr_t RelativeOrientationPtr_t;
    typedef constraints::RelativePositionPtr_t RelativePositionPtr_t;
    typedef constraints::RelativeTransformation RelativeTransformation;
    typedef constraints::RelativeTransformationPtr_t
    RelativeTransformationPtr_t;
    typedef model::value_type value_type;
    typedef model::size_type size_type;
    typedef model::Transform3f Transform3f;
    typedef model::vector_t vector_t;
    typedef model::vectorIn_t vectorIn_t;
    typedef model::vectorOut_t vectorOut_t;
    HPP_PREDEF_CLASS (ManipulationPlanner);
    typedef boost::shared_ptr < ManipulationPlanner > ManipulationPlannerPtr_t;
    HPP_PREDEF_CLASS (GraphPathValidation);
    typedef boost::shared_ptr < GraphPathValidation > GraphPathValidationPtr_t;
    HPP_PREDEF_CLASS (GraphSteeringMethod);
    typedef boost::shared_ptr < GraphSteeringMethod > GraphSteeringMethodPtr_t;
    typedef core::PathOptimizer PathOptimizer;
    typedef core::PathOptimizerPtr_t PathOptimizerPtr_t;
    HPP_PREDEF_CLASS (GraphOptimizer);
    typedef boost::shared_ptr < GraphOptimizer > GraphOptimizerPtr_t;
    HPP_PREDEF_CLASS (GraphNodeOptimizer);
    typedef boost::shared_ptr < GraphNodeOptimizer > GraphNodeOptimizerPtr_t;
    typedef core::PathProjectorPtr_t PathProjectorPtr_t;

    typedef std::vector <model::DevicePtr_t> Devices_t;
    typedef std::vector <ObjectPtr_t> Objects_t;
    typedef std::map <JointConstPtr_t, JointPtr_t> JointMap_t;
    typedef core::Constraint Constraint;
    typedef core::ConstraintPtr_t ConstraintPtr_t;
    typedef core::LockedJoint LockedJoint;
    typedef core::LockedJointPtr_t LockedJointPtr_t;
    typedef core::NumericalConstraint NumericalConstraint;
    typedef core::NumericalConstraintPtr_t NumericalConstraintPtr_t;
    typedef core::ConfigProjector ConfigProjector;
    typedef core::ConfigProjectorPtr_t ConfigProjectorPtr_t;
    HPP_PREDEF_CLASS (ConstraintSet);
    typedef boost::shared_ptr <ConstraintSet> ConstraintSetPtr_t;
    typedef core::DifferentiableFunctionPtr_t DifferentiableFunctionPtr_t;
    typedef core::ConfigurationShooter ConfigurationShooter;
    typedef core::ConfigurationShooterPtr_t ConfigurationShooterPtr_t;
    typedef core::ValidationReport ValidationReport;
    typedef core::PathValidationReportPtr_t PathValidationReportPtr_t;
    typedef core::matrix_t matrix_t;
    typedef core::matrixIn_t matrixIn_t;
    typedef core::matrixOut_t matrixOut_t;
    typedef core::size_type size_type;
    typedef core::value_type value_type;
    typedef core::vector3_t vector3_t;

    typedef std::list < NumericalConstraintPtr_t > NumericalConstraints_t;
    typedef std::pair< GripperPtr_t, HandlePtr_t> Grasp_t;
    typedef boost::shared_ptr <Grasp_t> GraspPtr_t;
    typedef std::map <NumericalConstraintPtr_t, GraspPtr_t> GraspsMap_t;

    typedef std::vector<core::vector3_t> Shape_t;
    typedef std::pair <JointPtr_t, Shape_t> JointAndShape_t;
    typedef std::list <JointAndShape_t> JointAndShapes_t;
    typedef HPP_MANIPULATION_DEPRECATED Shape_t Triangle;
    typedef HPP_MANIPULATION_DEPRECATED JointAndShape_t JointAndTriangle_t;
    typedef HPP_MANIPULATION_DEPRECATED JointAndShapes_t JointAndTriangles_t;
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_FWD_HH
