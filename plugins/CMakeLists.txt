# Copyright (c) 2019, 2020, CNRS
# Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
# Authors: Guilhem Saurel (guilhem.saurel@laas.fr)
#
# This file is part of hpp-manipulation.
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
# hpp-manipulation. If not, see <http://www.gnu.org/licenses/>.

INCLUDE(${HPP_CORE_CMAKE_PLUGIN})

HPP_ADD_PLUGIN(manipulation-spline-gradient-based
  SOURCES spline-gradient-based.cc
  LINK_DEPENDENCIES ${PROJECT_NAME}-gpl)

HPP_ADD_PLUGIN(end-effector-trajectory
  SOURCES end-effector-trajectory.cc
  LINK_DEPENDENCIES ${PROJECT_NAME})
