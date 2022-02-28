#
# Copyright (c) 2014, 2020 CNRS
# Authors: Florent Lamiraux, Guilhem Saurel
#
#

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
# DAMAGE.

CMAKE_MINIMUM_REQUIRED(VERSION 3.1)

SET(PROJECT_NAME hpp-manipulation)
SET(PROJECT_DESCRIPTION "Classes for manipulation planning.")

SET(PROJECT_USE_CMAKE_EXPORT TRUE)
SET(CXX_DISABLE_WERROR TRUE)

INCLUDE(cmake/hpp.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/test.cmake)

COMPUTE_PROJECT_ARGS(PROJECT_ARGS LANGUAGES CXX)
PROJECT(${PROJECT_NAME} ${PROJECT_ARGS})

LIST(APPEND PKG_CONFIG_ADDITIONAL_VARIABLES cmake_plugin)

# Ask Doxygen to create a tree view in html documentation
SET(DOXYGEN_TREEVIEW "NO" CACHE STRING "Set to YES to generate a tree view in the html documentation")

ADD_PROJECT_DEPENDENCY(Boost REQUIRED COMPONENTS regex)

ADD_PROJECT_DEPENDENCY("hpp-core" REQUIRED)
IF(BUILD_TESTING)
  FIND_PACKAGE(Boost REQUIRED COMPONENTS unit_test_framework)
  ADD_PROJECT_DEPENDENCY("example-robot-data" REQUIRED)
ENDIF()

CONFIG_FILES(doc/main.hh
  include/hpp/manipulation/package-config.hh)

SET(${PROJECT_NAME}_HEADERS
  ${CMAKE_BINARY_DIR}/include/hpp/manipulation/package-config.hh
  include/hpp/manipulation/fwd.hh
  include/hpp/manipulation/handle.hh
  include/hpp/manipulation/problem.hh
  include/hpp/manipulation/problem-solver.hh
  include/hpp/manipulation/device.hh
  include/hpp/manipulation/weighed-distance.hh
  include/hpp/manipulation/constraint-set.hh
  include/hpp/manipulation/roadmap.hh
  include/hpp/manipulation/roadmap-node.hh
  include/hpp/manipulation/connected-component.hh
  include/hpp/manipulation/leaf-connected-comp.hh
  include/hpp/manipulation/manipulation-planner.hh
  include/hpp/manipulation/graph-path-validation.hh
  include/hpp/manipulation/graph-optimizer.hh
  include/hpp/manipulation/graph/state.hh
  include/hpp/manipulation/graph/edge.hh
  include/hpp/manipulation/graph/state-selector.hh
  include/hpp/manipulation/graph/guided-state-selector.hh
  include/hpp/manipulation/graph/graph.hh
  include/hpp/manipulation/graph/statistics.hh
  include/hpp/manipulation/graph/graph-component.hh
  include/hpp/manipulation/graph/fwd.hh
  include/hpp/manipulation/graph/dot.hh
  include/hpp/manipulation/graph/helper.hh
  include/hpp/manipulation/graph/validation.hh

  include/hpp/manipulation/path-optimization/enforce-transition-semantic.hh
  include/hpp/manipulation/path-optimization/random-shortcut.hh
  include/hpp/manipulation/path-optimization/spline-gradient-based.hh

  include/hpp/manipulation/path-planner/end-effector-trajectory.hh

  include/hpp/manipulation/problem-target/state.hh

  include/hpp/manipulation/serialization.hh

  include/hpp/manipulation/steering-method/cross-state-optimization.hh
  include/hpp/manipulation/steering-method/fwd.hh
  include/hpp/manipulation/steering-method/graph.hh
  include/hpp/manipulation/steering-method/end-effector-trajectory.hh
  )

SET(${PROJECT_NAME}_SOURCES
  src/handle.cc
  src/manipulation-planner.cc
  src/problem-solver.cc
  src/roadmap.cc
  src/connected-component.cc
  src/leaf-connected-comp.cc
  src/constraint-set.cc
  src/roadmap-node.cc
  src/device.cc
  src/weighed-distance.cc
  src/problem.cc
  src/graph-path-validation.cc
  src/graph-optimizer.cc

  src/graph/state.cc
  src/graph/edge.cc
  src/graph/graph.cc
  src/graph/graph-component.cc
  src/graph/state-selector.cc
  src/graph/guided-state-selector.cc
  src/graph/statistics.cc
  src/graph/helper.cc

  src/graph/dot.cc
  src/graph/validation.cc

  src/path-optimization/random-shortcut.cc
  src/path-optimization/enforce-transition-semantic.cc

  src/path-planner/end-effector-trajectory.cc

  src/problem-target/state.cc

  src/serialization.cc

  src/steering-method/end-effector-trajectory.cc
  src/steering-method/cross-state-optimization.cc
  src/steering-method/graph.cc
  )

ADD_LIBRARY(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_HEADERS})
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:include>)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} hpp-core::hpp-core Boost::regex)

INSTALL(TARGETS ${PROJECT_NAME} EXPORT ${TARGETS_EXPORT_NAME} DESTINATION lib)

# GPL licenced part

SET(${PROJECT_NAME}_SOURCES_GPL
  src/path-optimization/spline-gradient-based.cc
  )

ADD_LIBRARY(${PROJECT_NAME}-gpl SHARED ${${PROJECT_NAME}_SOURCES_GPL})
TARGET_LINK_LIBRARIES(${PROJECT_NAME}-gpl ${PROJECT_NAME} hpp-core::hpp-core-gpl)

INSTALL(TARGETS ${PROJECT_NAME}-gpl EXPORT ${TARGETS_EXPORT_NAME} DESTINATION lib)

ADD_SUBDIRECTORY(plugins)
IF(BUILD_TESTING)
  ADD_SUBDIRECTORY(tests)
ENDIF()

PKG_CONFIG_APPEND_LIBS(${PROJECT_NAME})

INSTALL(FILES package.xml DESTINATION share/${PROJECT_NAME})
