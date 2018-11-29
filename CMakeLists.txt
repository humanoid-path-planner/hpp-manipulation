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

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(CXX_DISABLE_WERROR TRUE)
INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)

SET(PROJECT_NAME hpp-manipulation)
SET(PROJECT_URL "")
SET(PROJECT_DESCRIPTION "Classes for manipulation planning.")

SETUP_PROJECT()

# Activate hpp-util logging if requested
SET (HPP_DEBUG FALSE CACHE BOOL "trigger hpp-util debug output")
IF (HPP_DEBUG)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHPP_DEBUG")
ENDIF()
SET (HPP_BENCHMARK FALSE CACHE BOOL "trigger hpp-util benchmarking output")
IF (HPP_BENCHMARK)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHPP_ENABLE_BENCHMARK")
ENDIF()

# Activate test using UR5 if requested
SET (TEST_UR5 FALSE CACHE BOOL "Activate tests using ur5")

# Ask Doxygen to create a tree view in html documentation
SET(DOXYGEN_TREEVIEW "NO" CACHE STRING "Set to YES to generate a tree view in the html documentation")

SET(HPP_MANIPULATION_HAS_WHOLEBODY_STEP TRUE
  CACHE BOOL "Compile with dependency to hpp-wholebody-step")

SET(BOOST_BUILD_COMPONENTS unit_test_framework)
SET(BOOST_INSTALL_COMPONENTS regex)
SET(BOOST_COMPONENTS ${BOOST_INSTALL_COMPONENTS} ${BOOST_BUILD_COMPONENTS})

SEARCH_FOR_BOOST()
ADD_DOC_DEPENDENCY("hpp-pinocchio >= 4")
ADD_DOC_DEPENDENCY("hpp-fcl >= 0.4.3")
ADD_REQUIRED_DEPENDENCY("hpp-core >= 4")
ADD_REQUIRED_DEPENDENCY("hpp-constraints >= 4")
ADD_REQUIRED_DEPENDENCY("hpp-statistics >= 1.1")
IF (HPP_MANIPULATION_HAS_WHOLEBODY_STEP)
   ADD_REQUIRED_DEPENDENCY("hpp-wholebody-step >= 4")
ENDIF ()
IF (TEST_UR5)
  ADD_REQUIRED_DEPENDENCY("hpp_universal_robot")
ENDIF ()

CONFIG_FILES (doc/main.hh
  include/hpp/manipulation/package-config.hh)

SET (${PROJECT_NAME}_HEADERS
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
  include/hpp/manipulation/symbolic-planner.hh
  include/hpp/manipulation/manipulation-planner.hh
  include/hpp/manipulation/graph-path-validation.hh
  include/hpp/manipulation/graph-optimizer.hh
  include/hpp/manipulation/graph/node.hh
  include/hpp/manipulation/graph/state.hh
  include/hpp/manipulation/graph/edge.hh
  include/hpp/manipulation/graph/node-selector.hh
  include/hpp/manipulation/graph/state-selector.hh
  include/hpp/manipulation/graph/guided-node-selector.hh
  include/hpp/manipulation/graph/guided-state-selector.hh
  include/hpp/manipulation/graph/graph.hh
  include/hpp/manipulation/graph/statistics.hh
  include/hpp/manipulation/graph/graph-component.hh
  include/hpp/manipulation/graph/fwd.hh
  include/hpp/manipulation/graph/dot.hh
  include/hpp/manipulation/graph/helper.hh

  include/hpp/manipulation/path-optimization/small-steps.hh
  include/hpp/manipulation/path-optimization/enforce-transition-semantic.hh
  include/hpp/manipulation/path-optimization/spline-gradient-based.hh

  include/hpp/manipulation/problem-target/state.hh

  include/hpp/manipulation/steering-method/cross-state-optimization.hh
  include/hpp/manipulation/steering-method/fwd.hh
  include/hpp/manipulation/steering-method/graph.hh
  )

ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(tests)

# Add dependency toward hpp-manipulation library in pkg-config file.
PKG_CONFIG_APPEND_LIBS("hpp-manipulation")
PKG_CONFIG_APPEND_BOOST_LIBS(${BOOST_INSTALL_COMPONENTS})

SETUP_PROJECT_FINALIZE()
