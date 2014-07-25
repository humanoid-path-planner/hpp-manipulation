///
/// Copyright (c) 2014 CNRS
/// Authors: Florent Lamiraux
///
///
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
# include <hpp/constraints/fwd.hh>

namespace hpp {
  namespace manipulation {
    typedef model::Device Device;
    typedef model::DevicePtr_t DevicePtr_t;
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
    typedef constraints::RelativeOrientation RelativeOrientation;
    typedef constraints::RelativePosition RelativePosition;
    typedef constraints::RelativeOrientationPtr_t RelativeOrientationPtr_t;
    typedef constraints::RelativePositionPtr_t RelativePositionPtr_t;
    typedef constraints::RelativeTransformation RelativeTransformation;
    typedef constraints::RelativeTransformationPtr_t
    RelativeTransformationPtr_t;
    HPP_PREDEF_CLASS (Robot);
    typedef boost::shared_ptr <Robot> RobotPtr_t;
    typedef model::value_type value_type;
    typedef model::size_type size_type;
    typedef model::Transform3f Transform3f;
    typedef model::vector_t vector_t;
    typedef model::vectorIn_t vectorIn_t;
    typedef model::vectorOut_t vectorOut_t;
    HPP_PREDEF_CLASS (ManipulationPlanner);
    typedef boost::shared_ptr < ManipulationPlanner > ManipulationPlannerPtr_t;
    typedef core::ConnectedComponentPtr_t ConnectedComponentPtr_t;

    typedef std::vector <DevicePtr_t> Devices_t;
    typedef std::vector <ObjectPtr_t> Objects_t;
    typedef std::map <JointConstPtr_t, JointPtr_t> JointMap_t;
    typedef core::Constraint Constraint;
    typedef core::ConstraintPtr_t ConstraintPtr_t;
    typedef core::LockedDof LockedDof;
    typedef core::LockedDofPtr_t LockedDofPtr_t;
    typedef core::ConfigProjector ConfigProjector;
    typedef core::ConfigProjectorPtr_t ConfigProjectorPtr_t;
    typedef core::ConstraintSet ConstraintSet;
    typedef core::ConstraintSetPtr_t ConstraintSetPtr_t;
    typedef core::DifferentiableFunctionPtr_t DifferentiableFunctionPtr_t;
    
    typedef std::pair< GripperPtr_t, HandlePtr_t> Grasp_t;
    typedef boost::shared_ptr <Grasp_t> GraspPtr_t;
    typedef std::map <DifferentiableFunctionPtr_t, GraspPtr_t> GraspsMap_t;
    typedef std::map <std::string, LockedDofPtr_t> LockedDofConstraintMap_t;

    HPP_PREDEF_CLASS(MotionProjector);
    typedef boost::shared_ptr < MotionProjector > MotionProjectorPtr_t;
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_FWD_HH
