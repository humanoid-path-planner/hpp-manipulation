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

namespace hpp {
  namespace manipulation {
    HPP_PREDEF_CLASS (Device);
    typedef boost::shared_ptr <Device> DevicePtr_t;
    typedef boost::shared_ptr <const Device> DeviceConstPtr_t;
    typedef pinocchio::Joint Joint;
    typedef pinocchio::JointPtr_t JointPtr_t;
    typedef pinocchio::JointIndex JointIndex;
    typedef std::vector<JointIndex> JointIndices_t;
    typedef pinocchio::FrameIndex FrameIndex;
    typedef std::vector<pinocchio::FrameIndex> FrameIndices_t;
    typedef pinocchio::Configuration_t Configuration_t;
    typedef pinocchio::ConfigurationIn_t ConfigurationIn_t;
    typedef pinocchio::ConfigurationOut_t ConfigurationOut_t;
    typedef core::ConfigurationPtr_t ConfigurationPtr_t;
    typedef pinocchio::GripperPtr_t GripperPtr_t;
    typedef pinocchio::LiegroupElement LiegroupElement;
    typedef pinocchio::LiegroupSpace LiegroupSpace;
    typedef pinocchio::LiegroupSpacePtr_t LiegroupSpacePtr_t;
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
    typedef boost::shared_ptr <Problem> ProblemPtr_t;
    HPP_PREDEF_CLASS (Roadmap);
    typedef boost::shared_ptr <Roadmap> RoadmapPtr_t;
    HPP_PREDEF_CLASS (RoadmapNode);
    typedef RoadmapNode* RoadmapNodePtr_t;
    typedef std::vector<RoadmapNodePtr_t> RoadmapNodes_t;
    HPP_PREDEF_CLASS (ConnectedComponent);
    typedef boost::shared_ptr<ConnectedComponent> ConnectedComponentPtr_t; 
    HPP_PREDEF_CLASS (LeafConnectedComp);
    typedef boost::shared_ptr<LeafConnectedComp> LeafConnectedCompPtr_t;
    typedef boost::shared_ptr<const LeafConnectedComp>
    LeafConnectedCompConstPtr_t;
    typedef std::set<LeafConnectedCompPtr_t> LeafConnectedComps_t;
    HPP_PREDEF_CLASS (WeighedLeafConnectedComp);
    typedef boost::shared_ptr<WeighedLeafConnectedComp> WeighedLeafConnectedCompPtr_t;
    HPP_PREDEF_CLASS (WeighedDistance);
    typedef boost::shared_ptr<WeighedDistance> WeighedDistancePtr_t;
    typedef constraints::RelativeOrientation RelativeOrientation;
    typedef constraints::RelativePosition RelativePosition;
    typedef constraints::RelativeOrientationPtr_t RelativeOrientationPtr_t;
    typedef constraints::RelativePositionPtr_t RelativePositionPtr_t;
    typedef constraints::RelativeTransformation RelativeTransformation;
    typedef constraints::RelativeTransformationPtr_t
    RelativeTransformationPtr_t;
    typedef core::value_type value_type;
    typedef core::size_type size_type;
    typedef core::Transform3f Transform3f;
    typedef core::vector_t vector_t;
    typedef core::vectorIn_t vectorIn_t;
    typedef core::vectorOut_t vectorOut_t;
    HPP_PREDEF_CLASS (ManipulationPlanner);
    typedef boost::shared_ptr < ManipulationPlanner > ManipulationPlannerPtr_t;
    HPP_PREDEF_CLASS (GraphPathValidation);
    typedef boost::shared_ptr < GraphPathValidation > GraphPathValidationPtr_t;
    HPP_PREDEF_CLASS (SteeringMethod);
    typedef boost::shared_ptr < SteeringMethod > SteeringMethodPtr_t;
    typedef core::PathOptimizer PathOptimizer;
    typedef core::PathOptimizerPtr_t PathOptimizerPtr_t;
    HPP_PREDEF_CLASS (GraphOptimizer);
    typedef boost::shared_ptr < GraphOptimizer > GraphOptimizerPtr_t;
    HPP_PREDEF_CLASS (GraphNodeOptimizer);
    typedef boost::shared_ptr < GraphNodeOptimizer > GraphNodeOptimizerPtr_t;
    typedef core::PathProjectorPtr_t PathProjectorPtr_t;

    typedef std::vector <pinocchio::DevicePtr_t> Devices_t;
    typedef std::vector <ObjectPtr_t> Objects_t;
    typedef core::Constraint Constraint;
    typedef core::ConstraintPtr_t ConstraintPtr_t;
    typedef constraints::ImplicitPtr_t ImplicitPtr_t;
    typedef constraints::LockedJoint LockedJoint;
    typedef constraints::LockedJointPtr_t LockedJointPtr_t;
    typedef hpp::core::ComparisonTypes_t ComparisonTypes_t;
    typedef core::ConfigProjector ConfigProjector;
    typedef core::ConfigProjectorPtr_t ConfigProjectorPtr_t;
    HPP_PREDEF_CLASS (ConstraintSet);
    typedef boost::shared_ptr <ConstraintSet> ConstraintSetPtr_t;
    typedef core::DifferentiableFunctionPtr_t DifferentiableFunctionPtr_t;
    typedef core::ConfigurationShooter ConfigurationShooter;
    typedef core::ConfigurationShooterPtr_t ConfigurationShooterPtr_t;
    typedef core::ValidationReport ValidationReport;
    typedef core::PathValidationPtr_t PathValidationPtr_t;
    typedef core::PathValidationReportPtr_t PathValidationReportPtr_t;
    typedef core::matrix_t matrix_t;
    typedef core::matrixIn_t matrixIn_t;
    typedef core::matrixOut_t matrixOut_t;
    typedef core::size_type size_type;
    typedef core::value_type value_type;
    typedef core::vector3_t vector3_t;
    typedef core::matrix3_t matrix3_t;

    typedef core::Shape_t Shape_t;
    typedef core::JointAndShape_t JointAndShape_t;
    typedef core::JointAndShapes_t JointAndShapes_t;

    typedef std::list <std::string> StringList_t;

    namespace pathOptimization {
      HPP_PREDEF_CLASS (SmallSteps);
      typedef boost::shared_ptr < SmallSteps > SmallStepsPtr_t;
      HPP_PREDEF_CLASS (Keypoints);
      typedef boost::shared_ptr < Keypoints > KeypointsPtr_t;
    } // namespace pathOptimization

    namespace problemTarget {
      HPP_PREDEF_CLASS (State);
      typedef boost::shared_ptr < State > StatePtr_t;
    } // namespace problemTarget
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_FWD_HH
