# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).

## [Unreleased]

- [README] Fix hpp-statistics link

## [9.1.0] - 2026-10-08

- Added changelog
- [pathOptimization] Add ManipulationSpline
- ROS: example-robot-{data -> descriptions}
- [StatesPathFinder] Keep short transition projections local
- [Edge] Update continuous validation for grasps
- [path-planner] Document planner-specific solve hooks

## [9.0.2] - 2026-07-24



## [9.0.0] - 2026-07-13



## [7.0.0] - 2026-03-06



## [6.1.0] - 2025-10-23



## [6.0.0] - 2024-12-07

Changes in v6.0.0
- hpp-fcl dependency has been replaced by coal
- updates for coal v3


## [5.2.0] - 2024-10-09

Changes v5.2.0:
- TransitionPlanner: in planPath, call checkProblemAndForwardParameters
- nix: move package to nixpkgs
- ci:use https
- setup mergify


## [5.1.0] - 2024-07-02

Changes in v5.1.0:
- implement new function to discard non-solvable potential solution for
SPF algorithm
- fix compilation warning
- Nix: initial support
- updated tooling


## [5.0.0] - 2024-03-31

Changes in v5.0.0:
- Device::setRobotRootPosition invalidates all the device data
- Update to removal of hpp::pinocchio::ConfigurationPtr_t
- [GraphOptimizer] Fix constraints in Problem
- Remove call to deprecated methods
- [TransitionPlanner] Implement InStatePlanner from agimus-demos
- [EndEffectorTrajectory] Document steering method and path planner
- [steeringMethod::EndEffectorTrajectory] Throw when steering method fails
- New manipulation planning algorithm
- [Handle] Initialize maskComp_ in constructor
- [GraphOptimizer] Check that constraint is not empty
- update packaging
- update tooling


## [4.15.1] - 2023-01-20



## [4.14.0] - 2022-11-02



## [4.13.0] - 2022-05-31



## [4.12.0] - 2021-10-06

Changes in v4.12.0:
- Fix steeringMethod::CrossStateOptimization


## [4.11.0] - 2021-05-04

Changes in v4.11.0:
- [Handle] Use constraints with values in R3xSO(3) for grasp constraints
- Add GraphPathValidation::setSecurityMarginBetweenBodies
- Improve serialization
- Update to changes in hpp-core
- Remove dependency to hpp-wholebody-step


## [4.10.1] - 2020-09-24

Changes since v4.9.0:
* In graph::steeringMethod, if q1 == q2, the steering method calls the problem
  inner steering method. This avoids a failure if no loop transition has been
  set on the state containing q1.
* In class Edge,
  - different security margins for different pairs of links can
    be set for collision checking,
  - rename some methods for homogeneity with python bindings
    - applyConstraints -> generateTargetConfig
    - configConstraint -> targetConstraint,
    - from -> stateFrom, to -> stateTo,
* In class WaypointEdge,
  - specialize initialization.
* In class LevelSetEdge,
  - write documentation,
  - rename method applyConstraintsWithOffset into generateTargetConfigOnLeaf,
  - add read access to condition and parameterization constraints.
* In class GraphComponent,
  - add cost
* Add end-effector-tracjectory plugin.
* In class ManipulationPlanner,
  - call the problem PathValidation instead of the edge one.
* In class ProblemSolver
  - register contact constraints and complement in constraint graph.

## [4.10.0] - 2020-08-17



## [4.9.0] - 2020-04-29

Changes in v4.9.0:
- In graph::steeringMethod, if q1 == q2, the steering method calls the problem
  inner steering method. This avoids a failure if no loop transition has been
  set on the state containing q1.
- In class Edge,
  - different security margins for different pairs of links can
    be set for collision checking,
  - rename some methods for homogeneity with python bindings
    - applyConstraints -> generateTargetConfig
    - configConstraint -> targetConstraint,
    - from -> stateFrom, to -> stateTo.
- In class LevelSetEdge,
  - write documentation,
  - rename method applyConstraintsWithOffset into generateTargetConfigOnLeaf,
  - add read access to condition and parameterization constraints.
- In class GraphComponent,
  - add cost
- Add end-effector-tracjectory plugin.
- In class ManipulationPlanner,
  - call the problem PathValidation instead of the edge one.
- CMake Exports

## [4.8.0] - 2019-11-28

Changes since v4.7.0:
- Improve graph validation + fix dynamic cast issue.
- update CMake

## [4.7.0] - 2019-10-04

Changes since v4.6.0:
- remove deprecated methods and files
- [Graph] Create default StateSelector at construction.
- [graph] Handle locked joint as other constraints.
- Fix graph::Validate::validateGraph
- Add class hpp::manipulation::graph::Validation
- Add StateSelector::getWaypointStates
- [Doc] Fix links in main page.
- [CrossStateOptimization] Update documentation of class and NEWS file.
- [CrossStateOptimization] Do not take previous waypoint as initialization
- [CrossStateOptimization] Decouple waypoint computations
- [Graph] When initializing graph, clear constraint and complement container.
- [Graph] Expose member constraintsAndComplements_.


## [4.5.0] - 2019-04-25

Changes since v4.4.0:
- Remove Device container of frame indices.
- Update shared_ptr for typedef ProblemPtr_t
- [CMake] Update to plugin API change.
- Add EndEffectorTrajectory steering method.
- [CI] include conf from rainboard
- Fix mask vector when creating grasp on non-freeflyer


## [4.4.0] - 2019-03-19

Changes since v4.3.0:
- Update to pinocchio v2
- add GPL plugin


## [4.3.0] - 2019-01-31

- [CI] add .gitlab-ci.yml & badges


## [4.2.0] - 2018-10-11

Changes since v4.1:
- Refactor constraints
- Remove assert in WaypointEdge::build.
- In WaypointEdge, do not create paths between 2 successive equal confi

## [4.0] - 2018-03-22

From this version on, hpp now depends on pinocchio package for all computations of forward kinematics.

[Unreleased]: https://github.com/humanoid-path-planner/hpp-manipulation/compare/v9.1.0...HEAD
[9.1.0]: https://github.com/humanoid-path-planner/hpp-manipulation/compare/v9.0.2...v9.1.0
[9.0.2]: https://github.com/humanoid-path-planner/hpp-manipulation/compare/v9.0.0...v9.0.2
[9.0.0]: https://github.com/humanoid-path-planner/hpp-manipulation/compare/v7.0.0...v9.0.0
[7.0.0]: https://github.com/humanoid-path-planner/hpp-manipulation/compare/v6.1.0...v7.0.0
[6.1.0]: https://github.com/humanoid-path-planner/hpp-manipulation/compare/v6.0.0...v6.1.0
[6.0.0]: https://github.com/humanoid-path-planner/hpp-manipulation/compare/v5.2.0...v6.0.0
[5.2.0]: https://github.com/humanoid-path-planner/hpp-manipulation/compare/v5.1.0...v5.2.0
[5.1.0]: https://github.com/humanoid-path-planner/hpp-manipulation/compare/v5.0.0...v5.1.0
[5.0.0]: https://github.com/humanoid-path-planner/hpp-manipulation/compare/v4.15.1...v5.0.0
[4.15.1]: https://github.com/humanoid-path-planner/hpp-manipulation/compare/v4.14.0...v4.15.1
[4.14.0]: https://github.com/humanoid-path-planner/hpp-manipulation/compare/v4.13.0...v4.14.0
[4.13.0]: https://github.com/humanoid-path-planner/hpp-manipulation/compare/v4.12.0...v4.13.0
[4.12.0]: https://github.com/humanoid-path-planner/hpp-manipulation/compare/v4.11.0...v4.12.0
[4.11.0]: https://github.com/humanoid-path-planner/hpp-manipulation/compare/v4.10.1...v4.11.0
[4.10.1]: https://github.com/humanoid-path-planner/hpp-manipulation/compare/v4.10.0...v4.10.1
[4.10.0]: https://github.com/humanoid-path-planner/hpp-manipulation/compare/v4.9.0...v4.10.0
[4.9.0]: https://github.com/humanoid-path-planner/hpp-manipulation/compare/v4.8.0...v4.9.0
[4.8.0]: https://github.com/humanoid-path-planner/hpp-manipulation/compare/v4.7.0...v4.8.0
[4.7.0]: https://github.com/humanoid-path-planner/hpp-manipulation/compare/v4.5.0...v4.7.0
[4.5.0]: https://github.com/humanoid-path-planner/hpp-manipulation/compare/v4.4.0...v4.5.0
[4.4.0]: https://github.com/humanoid-path-planner/hpp-manipulation/compare/v4.3.0...v4.4.0
[4.3.0]: https://github.com/humanoid-path-planner/hpp-manipulation/compare/v4.2.0...v4.3.0
[4.2.0]: https://github.com/humanoid-path-planner/hpp-manipulation/compare/v4.0...v4.2.0
[4.0]: https://github.com/humanoid-path-planner/hpp-manipulation/releases/tag/v4.0
