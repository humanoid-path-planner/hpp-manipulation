#
# Copyright (c) 2014 CNRS
# Authors: Florent Lamiraux
#
#
# This file is part of hpp-manipulation
# hpp-manipulation is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# hpp-manipulation is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp-manipulation  If not, see
# <http://www.gnu.org/licenses/>.

SET(LIBRARY_NAME ${PROJECT_NAME})

SET(SOURCES
  axial-handle.cc
  handle.cc
  symbolic-planner.cc
  manipulation-planner.cc
  problem-solver.cc
  roadmap.cc
  connected-component.cc
  symbolic-component.cc
  constraint-set.cc
  roadmap-node.cc
  device.cc
  weighed-distance.cc
  problem.cc
  graph-path-validation.cc
  graph-steering-method.cc
  graph-optimizer.cc

  graph/state.cc
  graph/edge.cc
  graph/graph.cc
  graph/graph-component.cc
  graph/state-selector.cc
  graph/guided-state-selector.cc
  graph/statistics.cc
  graph/helper.cc

  graph/dot.cc

  path-optimization/config-optimization.cc
  path-optimization/keypoints.cc
  path-optimization/spline-gradient-based.cc

  problem-target/state.cc
  )

IF(HPP_WHOLEBODY_STEP_FOUND)
  SET(SOURCES
    ${SOURCES}
    path-optimization/small-steps.cc
    )
ENDIF(HPP_WHOLEBODY_STEP_FOUND)

ADD_LIBRARY(${LIBRARY_NAME} SHARED ${SOURCES})

PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-core)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-statistics)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-constraints)
IF(HPP_WHOLEBODY_STEP_FOUND)
  PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-wholebody-step)
ENDIF(HPP_WHOLEBODY_STEP_FOUND)

INSTALL(TARGETS ${LIBRARY_NAME} DESTINATION lib)
