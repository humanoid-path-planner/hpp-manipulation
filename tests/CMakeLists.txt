# Copyright 2012, 2013, 2014, 2020 CNRS-LAAS
#
# Authors: Mathieu Geisert, Guilhem Saurel
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

# Make Boost.Test generates the main function in test cases.
ADD_DEFINITIONS(-DBOOST_TEST_DYN_LINK -DBOOST_TEST_MAIN)

ADD_UNIT_TEST(test-constraintgraph test-constraintgraph.cc)
TARGET_LINK_LIBRARIES(test-constraintgraph ${PROJECT_NAME} Boost::unit_test_framework)
