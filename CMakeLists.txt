#
# Copyright (c) 2014, 2020 CNRS Authors: Florent Lamiraux, Guilhem Saurel
#

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# 1. Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

cmake_minimum_required(VERSION 3.1)

set(PROJECT_NAME hpp-manipulation)
set(PROJECT_DESCRIPTION "Classes for manipulation planning.")

set(PROJECT_USE_CMAKE_EXPORT TRUE)
set(CXX_DISABLE_WERROR TRUE)

include(cmake/hpp.cmake)
include(cmake/boost.cmake)
include(cmake/test.cmake)

compute_project_args(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})

list(APPEND PKG_CONFIG_ADDITIONAL_VARIABLES cmake_plugin)

# Ask Doxygen to create a tree view in html documentation
set(DOXYGEN_TREEVIEW
    "NO"
    CACHE STRING "Set to YES to generate a tree view in the html documentation")

add_project_dependency(Boost REQUIRED COMPONENTS regex)

add_project_dependency("hpp-core" REQUIRED)
if(BUILD_TESTING)
  find_package(Boost REQUIRED COMPONENTS unit_test_framework)
  add_project_dependency("example-robot-data" REQUIRED)
endif()

config_files(doc/main.hh include/hpp/manipulation/package-config.hh)

set(${PROJECT_NAME}_HEADERS
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
    include/hpp/manipulation/path-planner/in-state-path.hh
    include/hpp/manipulation/path-planner/states-path-finder.hh
    include/hpp/manipulation/problem-target/state.hh
    include/hpp/manipulation/serialization.hh
    include/hpp/manipulation/steering-method/cross-state-optimization.hh
    include/hpp/manipulation/steering-method/fwd.hh
    include/hpp/manipulation/steering-method/graph.hh
    include/hpp/manipulation/steering-method/end-effector-trajectory.hh)

set(${PROJECT_NAME}_SOURCES
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
    src/path-planner/states-path-finder.cc
    src/problem-target/state.cc
    src/serialization.cc
    src/steering-method/end-effector-trajectory.cc
    src/steering-method/cross-state-optimization.cc
    src/steering-method/graph.cc)

add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SOURCES}
                                   ${${PROJECT_NAME}_HEADERS})
target_include_directories(${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} hpp-core::hpp-core Boost::regex)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${TARGETS_EXPORT_NAME}
  DESTINATION lib)

# GPL licenced part

set(${PROJECT_NAME}_SOURCES_GPL src/path-optimization/spline-gradient-based.cc)

add_library(${PROJECT_NAME}-gpl SHARED ${${PROJECT_NAME}_SOURCES_GPL})
target_link_libraries(${PROJECT_NAME}-gpl ${PROJECT_NAME}
                      hpp-core::hpp-core-gpl)

install(
  TARGETS ${PROJECT_NAME}-gpl
  EXPORT ${TARGETS_EXPORT_NAME}
  DESTINATION lib)

add_subdirectory(plugins)
if(BUILD_TESTING)
  add_subdirectory(tests)
endif()

pkg_config_append_libs(${PROJECT_NAME})

install(FILES package.xml DESTINATION share/${PROJECT_NAME})
