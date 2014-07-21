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

ADD_LIBRARY(${LIBRARY_NAME} SHARED
  axial-handle.cc
  handle.cc
  problem-solver.cc
  robot.cc
  manipulation-planner.cc

  graph/node.cc
  graph/edge.cc
  graph/graph.cc
  graph/node-selector.cc
)

PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-core)

INSTALL(TARGETS ${LIBRARY_NAME} DESTINATION lib)
