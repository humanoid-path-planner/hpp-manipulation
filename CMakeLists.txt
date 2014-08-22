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

# Activate test using UR5 if requested
SET (TEST_UR5 FALSE CACHE BOOL "Activate tests using ur5")

SEARCH_FOR_BOOST()
ADD_DOC_DEPENDENCY(hpp-model >= 3.0.0)
ADD_REQUIRED_DEPENDENCY(hpp-core >= 3.0.0)
ADD_REQUIRED_DEPENDENCY(hpp-constraints >= 3.0.0)
ADD_REQUIRED_DEPENDENCY(hpp-statistics >= 0.1)
IF (TEST_UR5)
  ADD_REQUIRED_DEPENDENCY(hpp-model-urdf >= 3.0.0)
ENDIF ()

SET (${PROJECT_NAME}_HEADERS
  include/hpp/manipulation/fwd.hh
  include/hpp/manipulation/axial-handle.hh
  include/hpp/manipulation/handle.hh
  include/hpp/manipulation/object.hh
  include/hpp/manipulation/problem.hh
  include/hpp/manipulation/problem-solver.hh
  include/hpp/manipulation/robot.hh
  include/hpp/manipulation/manipulation-planner.hh
  include/hpp/manipulation/graph-path-validation.hh
  include/hpp/manipulation/graph-steering-method.hh
  include/hpp/manipulation/graph/node.hh
  include/hpp/manipulation/graph/edge.hh
  include/hpp/manipulation/graph/node-selector.hh
  include/hpp/manipulation/graph/graph.hh
  include/hpp/manipulation/graph/graph-component.hh
  include/hpp/manipulation/graph/fwd.hh
  )

# Add dependency toward hpp-model library in pkg-config file.
PKG_CONFIG_APPEND_LIBS("hpp-manipulation")

ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(tests)

SETUP_PROJECT_FINALIZE()
