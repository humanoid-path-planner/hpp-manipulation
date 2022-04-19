// Copyright (c) 2014 CNRS
// Author: Florent Lamiraux, Joseph Mirabel
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

#ifndef HPP_MANIPULATION_PROBLEM_SOLVER_HH
#define HPP_MANIPULATION_PROBLEM_SOLVER_HH

#include <hpp/core/container.hh>
#include <hpp/core/problem-solver.hh>
#include <hpp/pinocchio/device.hh>
#include <map>

#include "hpp/manipulation/constraint-set.hh"
#include "hpp/manipulation/deprecated.hh"
#include "hpp/manipulation/device.hh"
#include "hpp/manipulation/fwd.hh"
#include "hpp/manipulation/graph/fwd.hh"

namespace hpp {
namespace manipulation {
class HPP_MANIPULATION_DLLAPI ProblemSolver : public core::ProblemSolver {
 public:
  typedef core::ProblemSolver parent_t;
  typedef std::vector<std::string> Names_t;

  /// Destructor
  virtual ~ProblemSolver() {}

  ProblemSolver();

  static ProblemSolverPtr_t create();

  /// Set robot
  /// Check that robot is of type hpp::manipulation::Device
  virtual void robot(const core::DevicePtr_t& robot) {
    robot_ = HPP_DYNAMIC_PTR_CAST(Device, robot);
    assert(robot_);
    parent_t::robot(robot);
  }

  /// Get robot
  const DevicePtr_t& robot() const { return robot_; }

  /// \name Constraint graph
  /// \{

  /// Set the constraint graph
  void constraintGraph(const std::string& graph);

  /// Get the constraint graph
  graph::GraphPtr_t constraintGraph() const;

  /// Should be called before any call on the graph is made.
  void initConstraintGraph();
  /// \}

  /// Create placement constraint
  /// \param name name of the placement constraint,
  /// \param triangleName name of the first list of triangles,
  /// \param envContactName name of the second list of triangles.
  /// \param margin see hpp::constraints::ConvexShapeContact::setNormalMargin
  ///
  void createPlacementConstraint(const std::string& name,
                                 const StringList_t& surface1,
                                 const StringList_t& surface2,
                                 const value_type& margin = 1e-4);

  /// Create pre-placement constraint
  /// \param name name of the placement constraint,
  /// \param triangleName name of the first list of triangles,
  /// \param envContactName name of the second list of triangles.
  /// \param width approaching distance.
  /// \param margin see hpp::constraints::ConvexShapeContact::setNormalMargin
  ///
  void createPrePlacementConstraint(const std::string& name,
                                    const StringList_t& surface1,
                                    const StringList_t& surface2,
                                    const value_type& width,
                                    const value_type& margin = 1e-4);

  /// Create the grasp constraint and its complement
  /// \param name name of the grasp constraint,
  /// \param gripper gripper's name
  /// \param handle handle's name
  ///
  /// Two constraints are created:
  /// - "name" corresponds to the grasp constraint.
  /// - "name/complement" corresponds to the complement.
  void createGraspConstraint(const std::string& name,
                             const std::string& gripper,
                             const std::string& handle);

  /// Create pre-grasp constraint
  /// \param name name of the grasp constraint,
  /// \param gripper gripper's name
  /// \param handle handle's name
  ///
  void createPreGraspConstraint(const std::string& name,
                                const std::string& gripper,
                                const std::string& handle);

  virtual void pathValidationType(const std::string& type,
                                  const value_type& tolerance);

  /// Create a new problem.
  virtual void resetProblem();

  /// Create a new Roadmap
  virtual void resetRoadmap();

  /// Get pointer to problem
  ProblemPtr_t problem() const { return problem_; }

  void setTargetState(const graph::StatePtr_t state);

  core::Container<graph::GraphPtr_t> graphs;

  ConstraintsAndComplements_t constraintsAndComplements;

 protected:
  virtual void initializeProblem(ProblemPtr_t problem);

 private:
  /// Keep track of the created components in order to retrieve them
  /// easily.
  graph::GraphComponents_t components_;

  DevicePtr_t robot_;
  /// The pointer should point to the same object as core::Problem.
  ProblemPtr_t problem_;
  graph::GraphPtr_t constraintGraph_;
};  // class ProblemSolver
}  // namespace manipulation
}  // namespace hpp

#endif  // HPP_MANIPULATION_PROBLEM_SOLVER_HH
