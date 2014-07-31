// Copyright (c) 2014, LAAS-CNRS
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//
// This file is part of hpp-manipulation.
// hpp-manipulation is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-manipulation is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-manipulation. If not, see <http://www.gnu.org/licenses/>.

#include <hpp/core/problem.hh>

#include "hpp/manipulation/robot.hh"
#include "hpp/manipulation/graph/graph.hh"
#include "hpp/manipulation/fwd.hh"

namespace hpp {
  namespace manipulation {
    class HPP_MANIPULATION_DLLAPI Problem : public core::Problem
    {
      public:
        /// Constructor
        Problem (RobotPtr_t robot) : core::Problem (robot),
        graph_()
        {
        }

        /// Set the graph of constraints
        void constraintGraph (const graph::GraphPtr_t& graph)
        {
          graph_ = graph;
        }

        /// Get the graph of constraints
        graph::GraphPtr_t constraintGraph () const
        {
          return graph_;
        }

      private:
        /// The graph of constraints
        graph::GraphPtr_t graph_;
    };
  } // namespace manipulation
} // namespace hpp
