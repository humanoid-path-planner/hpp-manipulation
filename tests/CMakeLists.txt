# Copyright 2012, 2013, 2014 CNRS-LAAS
#
# Author: Mathieu Geisert
#
# This file is part of hpp-core
# hpp-manipulation is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# hpp-manipulation is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Lesser Public License for more details.
# You should have received a copy of the GNU Lesser General Public License
# along with hpp-manipulation  If not, see <http://www.gnu.org/licenses/>.

INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})

# Make Boost.Test generates the main function in test cases.
ADD_DEFINITIONS(-DBOOST_TEST_DYN_LINK -DBOOST_TEST_MAIN)

# ADD_TESTCASE(NAME)
# ------------------------
#
# Define a test named `NAME'.
#
# This macro will create a binary from `NAME.cc', link it against
# Boost and add it to the test suite.
#
MACRO(ADD_TESTCASE NAME GENERATED)
  IF (${GENERATED} STREQUAL TRUE)
    ADD_EXECUTABLE(${NAME} ${CMAKE_CURRENT_BINARY_DIR}/${NAME}.cc)
  ELSE()
    ADD_EXECUTABLE(${NAME} ${NAME}.cc)
  ENDIF()
  ADD_TEST(${NAME} ${RUNTIME_OUTPUT_DIRECTORY}/${NAME})

  PKG_CONFIG_USE_DEPENDENCY (${NAME} hpp-constraints)
  # Link against Boost and project library.
  TARGET_LINK_LIBRARIES(${NAME}
    ${PROJECT_NAME}
    ${Boost_LIBRARIES}
    )

ENDMACRO(ADD_TESTCASE)

IF (TEST_UR5)
  ADD_TESTCASE (test-constraintgraph FALSE)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DTEST_UR5")
ENDIF ()
